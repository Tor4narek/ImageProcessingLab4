#if __has_include(<embree3/rtcore.h>)
#define USE_EMBREE3_API 1
#include <embree3/rtcore.h>
#include <embree3/rtcore_ray.h>
#elif __has_include(<embree4/rtcore.h>)
#define USE_EMBREE3_API 0
#include <embree4/rtcore.h>
#include <embree4/rtcore_ray.h>
#include <embree4/rtcore_scene.h>
#else
#error "Заголовки Embree не найдены. Установите пакет разработки Embree."
#endif

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

struct Vec3 {
  // Простая структура для 3D-векторов и RGB-цветов.
  // В этой работе один и тот же тип используется и для координат, и для цвета.
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  Vec3() = default;
  Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

  Vec3 operator+(const Vec3 &rhs) const { return Vec3(x + rhs.x, y + rhs.y, z + rhs.z); }
  Vec3 operator-(const Vec3 &rhs) const { return Vec3(x - rhs.x, y - rhs.y, z - rhs.z); }
  Vec3 operator-() const { return Vec3(-x, -y, -z); }
  Vec3 operator*(double s) const { return Vec3(x * s, y * s, z * s); }
  Vec3 operator/(double s) const { return Vec3(x / s, y / s, z / s); }

  Vec3 &operator+=(const Vec3 &rhs) {
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
    return *this;
  }
};

Vec3 operator*(double s, const Vec3 &v) { return v * s; }
Vec3 operator*(const Vec3 &a, const Vec3 &b) { return Vec3(a.x * b.x, a.y * b.y, a.z * b.z); }

double Dot(const Vec3 &a, const Vec3 &b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

Vec3 Cross(const Vec3 &a, const Vec3 &b) {
  return Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

double Length(const Vec3 &v) { return std::sqrt(Dot(v, v)); }

Vec3 Normalize(const Vec3 &v) {
  const double len = Length(v);
  if (len <= 1e-12) {
    return Vec3(0.0, 0.0, 0.0);
  }
  return v / len;
}

Vec3 Reflect(const Vec3 &I, const Vec3 &N) {
  return I - 2.0 * Dot(I, N) * N;
}

double Clamp01(double x) {
  return std::max(0.0, std::min(1.0, x));
}

Vec3 Clamp01(const Vec3 &v) {
  return Vec3(Clamp01(v.x), Clamp01(v.y), Clamp01(v.z));
}

struct Ray {
  Vec3 origin;
  Vec3 direction;
};

struct Material {
  // albedo     - базовый цвет материала (RGB).
  // kd         - коэффициент диффузного отражения (Ламберт).
  // ks         - коэффициент зеркально-бликового отражения (Фонг).
  // shininess  - "острота" блика по Фонгу.
  // mirror     - доля идеального зеркального отражения (рекурсивный луч).
  Vec3 albedo;
  double kd = 0.0;
  double ks = 0.0;
  double shininess = 1.0;
  double mirror = 0.0;
};

struct Light {
  // Точечный источник света.
  Vec3 position;
  Vec3 intensity;
};

struct Camera {
  // Параметры простой pinhole-камеры.
  Vec3 position;
  Vec3 target;
  Vec3 up;
  double fovDeg = 45.0;
};

struct TriangleMeshData {
  std::vector<Vec3> vertices;
  std::vector<std::array<unsigned, 3>> indices;
};

struct HitInfo {
  // Информация о пересечении луча со сценой.
  bool hit = false;
  float t = 0.0f;
  unsigned geomID = RTC_INVALID_GEOMETRY_ID;
  Vec3 Ng;
};

void DeviceErrorFunction(void *, enum RTCError error, const char *str) {
  std::cerr << "Ошибка Embree " << error << ": " << (str ? str : "(без подробностей)") << "\n";
}

TriangleMeshData MakeQuad(const Vec3 &a, const Vec3 &b, const Vec3 &c, const Vec3 &d) {
  TriangleMeshData mesh;
  mesh.vertices = {a, b, c, d};
  mesh.indices = {{0, 1, 2}, {0, 2, 3}};
  return mesh;
}

TriangleMeshData MakeBox(const Vec3 &pMin, const Vec3 &pMax) {
  TriangleMeshData mesh;
  mesh.vertices = {
      Vec3(pMin.x, pMin.y, pMin.z),  // 0
      Vec3(pMax.x, pMin.y, pMin.z),  // 1
      Vec3(pMax.x, pMin.y, pMax.z),  // 2
      Vec3(pMin.x, pMin.y, pMax.z),  // 3
      Vec3(pMin.x, pMax.y, pMin.z),  // 4
      Vec3(pMax.x, pMax.y, pMin.z),  // 5
      Vec3(pMax.x, pMax.y, pMax.z),  // 6
      Vec3(pMin.x, pMax.y, pMax.z)   // 7
  };

  // 12 корректных треугольников (без вырожденных индексов).
  mesh.indices = {
      {0, 1, 2}, {0, 2, 3},  // низ
      {4, 6, 5}, {4, 7, 6},  // верх
      {0, 4, 5}, {0, 5, 1},  // передняя грань
      {1, 5, 6}, {1, 6, 2},  // правая грань
      {2, 6, 7}, {2, 7, 3},  // задняя грань
      {3, 7, 4}, {3, 4, 0}   // левая грань
  };
  return mesh;
}

TriangleMeshData MakeSphere(const Vec3 &center, double radius, int stacks, int slices) {
  TriangleMeshData mesh;
  mesh.vertices.reserve((stacks + 1) * (slices + 1));
  mesh.indices.reserve(stacks * slices * 2);

  const double pi = 3.14159265358979323846;
  for (int i = 0; i <= stacks; ++i) {
    const double v = static_cast<double>(i) / static_cast<double>(stacks);
    const double phi = pi * v;
    const double y = std::cos(phi);
    const double ringR = std::sin(phi);
    for (int j = 0; j <= slices; ++j) {
      const double u = static_cast<double>(j) / static_cast<double>(slices);
      const double theta = 2.0 * pi * u;
      const double x = std::cos(theta) * ringR;
      const double z = std::sin(theta) * ringR;
      mesh.vertices.push_back(center + radius * Vec3(x, y, z));
    }
  }

  const int ring = slices + 1;
  for (int i = 0; i < stacks; ++i) {
    for (int j = 0; j < slices; ++j) {
      const unsigned a = static_cast<unsigned>(i * ring + j);
      const unsigned b = static_cast<unsigned>((i + 1) * ring + j);
      const unsigned c = static_cast<unsigned>(a + 1);
      const unsigned d = static_cast<unsigned>(b + 1);
      mesh.indices.push_back({a, b, c});
      mesh.indices.push_back({c, b, d});
    }
  }
  return mesh;
}

RTCGeometry CreateEmbreeGeometry(RTCDevice device, const TriangleMeshData &mesh) {
  RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);

  float *vertices = static_cast<float *>(rtcSetNewGeometryBuffer(
      geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3 * sizeof(float), mesh.vertices.size()));
  unsigned *indices = static_cast<unsigned *>(rtcSetNewGeometryBuffer(
      geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, 3 * sizeof(unsigned), mesh.indices.size()));

  for (size_t i = 0; i < mesh.vertices.size(); ++i) {
    vertices[3 * i + 0] = static_cast<float>(mesh.vertices[i].x);
    vertices[3 * i + 1] = static_cast<float>(mesh.vertices[i].y);
    vertices[3 * i + 2] = static_cast<float>(mesh.vertices[i].z);
  }
  for (size_t i = 0; i < mesh.indices.size(); ++i) {
    indices[3 * i + 0] = mesh.indices[i][0];
    indices[3 * i + 1] = mesh.indices[i][1];
    indices[3 * i + 2] = mesh.indices[i][2];
  }

  rtcCommitGeometry(geom);
  return geom;
}

void MapMaterialForGeom(std::vector<unsigned> &geomToMaterial, unsigned geomID, unsigned materialID) {
  if (geomID >= geomToMaterial.size()) {
    geomToMaterial.resize(geomID + 1, std::numeric_limits<unsigned>::max());
  }
  geomToMaterial[geomID] = materialID;
}

void AttachMesh(RTCDevice device, RTCScene scene, const TriangleMeshData &mesh, unsigned materialID,
                std::vector<unsigned> &geomToMaterial) {
  RTCGeometry geom = CreateEmbreeGeometry(device, mesh);
  const unsigned geomID = rtcAttachGeometry(scene, geom);
  MapMaterialForGeom(geomToMaterial, geomID, materialID);
  rtcReleaseGeometry(geom);
}

bool IntersectClosest(RTCScene scene, const Ray &ray, HitInfo &outHit, float tNear) {
  // Функция ищет ближайшее пересечение луча со сценой через Embree.
  RTCRayHit rayhit{};
  rayhit.ray.org_x = static_cast<float>(ray.origin.x);
  rayhit.ray.org_y = static_cast<float>(ray.origin.y);
  rayhit.ray.org_z = static_cast<float>(ray.origin.z);
  rayhit.ray.dir_x = static_cast<float>(ray.direction.x);
  rayhit.ray.dir_y = static_cast<float>(ray.direction.y);
  rayhit.ray.dir_z = static_cast<float>(ray.direction.z);
  rayhit.ray.tnear = tNear;
  rayhit.ray.tfar = std::numeric_limits<float>::infinity();
  rayhit.ray.mask = 0xFFFFFFFF;
  rayhit.ray.flags = 0;
  rayhit.ray.time = 0.0f;
  rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
  rayhit.hit.primID = RTC_INVALID_GEOMETRY_ID;
  rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

#if USE_EMBREE3_API
  RTCIntersectContext context;
  rtcInitIntersectContext(&context);
  rtcIntersect1(scene, &context, &rayhit);
#else
  RTCIntersectArguments args;
  rtcInitIntersectArguments(&args);
  rtcIntersect1(scene, &rayhit, &args);
#endif

  if (rayhit.hit.geomID == RTC_INVALID_GEOMETRY_ID) {
    return false;
  }

  outHit.hit = true;
  outHit.t = rayhit.ray.tfar;
  outHit.geomID = rayhit.hit.geomID;
  outHit.Ng = Vec3(rayhit.hit.Ng_x, rayhit.hit.Ng_y, rayhit.hit.Ng_z);
  return true;
}

bool IsOccluded(RTCScene scene, const Vec3 &origin, const Vec3 &dirToLight, float maxDistance, float eps) {
  // Теневой луч: проверяем, есть ли препятствие между точкой и источником света.
  RTCRay shadow{};
  shadow.org_x = static_cast<float>(origin.x);
  shadow.org_y = static_cast<float>(origin.y);
  shadow.org_z = static_cast<float>(origin.z);
  shadow.dir_x = static_cast<float>(dirToLight.x);
  shadow.dir_y = static_cast<float>(dirToLight.y);
  shadow.dir_z = static_cast<float>(dirToLight.z);
  shadow.tnear = eps;
  shadow.tfar = maxDistance - eps;
  shadow.mask = 0xFFFFFFFF;
  shadow.flags = 0;
  shadow.time = 0.0f;

  if (shadow.tfar <= shadow.tnear) {
    return false;
  }

#if USE_EMBREE3_API
  RTCIntersectContext context;
  rtcInitIntersectContext(&context);
  rtcOccluded1(scene, &context, &shadow);
#else
  RTCOccludedArguments args;
  rtcInitOccludedArguments(&args);
  rtcOccluded1(scene, &shadow, &args);
#endif

  // Embree помечает окклюзию отрицательным значением tfar.
  return shadow.tfar < 0.0f;
}

Vec3 TraceRay(RTCScene scene, const std::vector<Material> &materials, const std::vector<unsigned> &geomToMaterial,
              const Light &light, const Vec3 &origin, const Vec3 &direction, int depth, float eps) {
  // Главная функция трассировки:
  // 1) находим пересечение,
  // 2) считаем локальное освещение,
  // 3) при необходимости добавляем зеркальное рекурсивное отражение.
  HitInfo hit;
  if (!IntersectClosest(scene, Ray{origin, direction}, hit, eps)) {
    // Если луч никуда не попал, возвращаем цвет фона (черный).
    return Vec3(0.0, 0.0, 0.0);
  }

  Vec3 N = Normalize(hit.Ng);
  if (Dot(N, direction) > 0.0) {
    // Разворачиваем нормаль "к камере", чтобы освещение считалось корректно.
    N = -N;
  }

  const Vec3 P = origin + static_cast<double>(hit.t) * direction;

  if (hit.geomID >= geomToMaterial.size()) {
    return Vec3(0.0, 0.0, 0.0);
  }
  const unsigned materialID = geomToMaterial[hit.geomID];
  if (materialID >= materials.size()) {
    return Vec3(0.0, 0.0, 0.0);
  }
  const Material &mat = materials[materialID];

  // В рамках ТЗ считаем только локальное освещение от источника (без отдельного ambient-члена).
  Vec3 color(0.0, 0.0, 0.0);
  const Vec3 toLight = light.position - P;
  const double dist2 = Dot(toLight, toLight);
  const double dist = std::sqrt(std::max(dist2, 1e-12));
  const Vec3 L = toLight / dist;
  const double nDotL = std::max(0.0, Dot(N, L));

  // Теневой луч имеет смысл только если поверхность освещается источником (nDotL > 0).
  if (nDotL > 0.0) {
    // Для теневого луча используем более маленький сдвиг, чем для вторичных лучей,
    // чтобы не получать отрыв тени от объекта (peter-panning).
    const float shadowEps = std::max(1e-6f, eps * 0.1f);
    const Vec3 shadowOrigin = P + static_cast<double>(shadowEps) * N;
    const bool occluded = IsOccluded(scene, shadowOrigin, L, static_cast<float>(dist), shadowEps);
    if (!occluded) {

      // Диффузная составляющая Ламберта.
      const Vec3 diffuse = mat.kd * (mat.albedo * light.intensity) * (nDotL / std::max(dist2, 1e-12));

      // Зеркально-бликовая составляющая Фонга.
      const Vec3 V = Normalize(-direction);
      const Vec3 R = Normalize(Reflect(-L, N));
      const double rDotV = std::max(0.0, Dot(R, V));
      const Vec3 specular =
          mat.ks * light.intensity * (std::pow(rDotV, mat.shininess) / std::max(dist2, 1e-12));

      color += diffuse + specular;
    }
    // Если точка в тени, прямой вклад источника не добавляется.
  }

  if (depth > 0 && mat.mirror > 0.0) {
    // Идеальное зеркальное отражение:
    // строим отраженный луч и рекурсивно добавляем его вклад.
    const Vec3 reflectionDir = Normalize(Reflect(direction, N));
    // Смещение в сторону, куда действительно уходит отраженный луч.
    const double reflectSign = Dot(reflectionDir, N) >= 0.0 ? 1.0 : -1.0;
    const Vec3 reflectionOrigin = P + (reflectSign * eps) * N;
    color += mat.mirror * TraceRay(scene, materials, geomToMaterial, light, reflectionOrigin, reflectionDir, depth - 1,
                                   eps);
  }

  return color;
}

bool SavePPM(const std::string &filename, int width, int height, const std::vector<Vec3> &pixels) {
  // Сохраняем изображение в текстовом формате PPM (P3):
  // именно поэтому внутри файла видно числа.
  std::ofstream out(filename);
  if (!out) {
    return false;
  }

  out << "P3\n" << width << " " << height << "\n255\n";
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const Vec3 c = pixels[y * width + x];
      const int r = static_cast<int>(std::round(255.0 * c.x));
      const int g = static_cast<int>(std::round(255.0 * c.y));
      const int b = static_cast<int>(std::round(255.0 * c.z));
      out << r << " " << g << " " << b << "\n";
    }
  }
  return true;
}

int main() {
  // ---- Параметры лабораторной, которые легко менять ----
  const int width = 900;
  const int height = 900;
  const int maxRecursionDepth = 3;
  const float eps = 5e-5f;          // Минимально достаточный сдвиг лучей для борьбы с самопересечением.
  const double exposure = 1.50;     // Небольшое осветление средних тонов.
  // Антиалиасинг: количество подвыборок на пиксель по X и Y.
  // Итоговое число лучей на пиксель = aaSamplesX * aaSamplesY.
  const int aaSamplesX = 2;
  const int aaSamplesY = 2;
  const int samplesPerPixel = aaSamplesX * aaSamplesY;
  // High-poly объект: сфера с ~4000 треугольников.
  // Количество треугольников для сферы = stacks * slices * 2.
  const int hiPolySphereStacks = 45;
  const int hiPolySphereSlices = 45;  // 45 * 45 * 2 = 4050 треугольников.
  const std::string outputFile = "output.ppm";

  const Camera camera{
      Vec3(0.0, 1.0, 3.2),   // положение камеры
      Vec3(0.0, 1.0, 0.0),   // точка, куда смотрит камера
      Vec3(0.0, 1.0, 0.0),   // вектор "вверх"
      45.0                   // угол обзора (в градусах)
  };
  const Light light{
      Vec3(0.0, 1.78, -0.2),    // источник смещен вглубь сцены: тень визуально стартует ближе к кубу
      Vec3(120.0, 120.0, 120.0) // интенсивность под новое положение источника
  };

  // Учебная реализация обратной трассировки лучей/путей
  // с локальным освещением (Ламберт + Фонг), тенями и рекурсивным зеркальным отражением.

  RTCDevice device = rtcNewDevice(nullptr);
  if (device == nullptr) {
    std::cerr << "Не удалось создать устройство Embree.\n";
    return 1;
  }
  rtcSetDeviceErrorFunction(device, DeviceErrorFunction, nullptr);

  RTCScene scene = rtcNewScene(device);

  std::vector<Material> materials;
  // Набор материалов сцены.
  // Порядок важен: ниже каждому geomID сопоставляется materialID.
  const unsigned matFloor = static_cast<unsigned>(materials.size());
  // Для пола отключаем зеркальный блик (ks=0), чтобы не появлялись "ложные" светлые пятна рядом с тенью.
  materials.push_back(Material{Vec3(0.75, 0.75, 0.75), 0.95, 0.0, 8.0, 0.0});
  const unsigned matWhite = static_cast<unsigned>(materials.size());
  // Стены и потолок делаем матовыми (ks=0), чтобы убрать паразитный пересвет.
  materials.push_back(Material{Vec3(0.75, 0.75, 0.75), 0.92, 0.0, 12.0, 0.0});
  const unsigned matRed = static_cast<unsigned>(materials.size());
  materials.push_back(Material{Vec3(0.85, 0.2, 0.2), 0.9, 0.1, 24.0, 0.0});
  const unsigned matGreen = static_cast<unsigned>(materials.size());
  materials.push_back(Material{Vec3(0.2, 0.85, 0.2), 0.9, 0.1, 24.0, 0.0});
  const unsigned matGlossy = static_cast<unsigned>(materials.size());
  materials.push_back(Material{Vec3(0.95, 0.95, 0.98), 0.72, 0.45, 80.0, 0.12});
  const unsigned matMirror = static_cast<unsigned>(materials.size());
  materials.push_back(Material{Vec3(0.95, 0.95, 0.95), 0.35, 0.18, 32.0, 0.45});

  std::vector<unsigned> geomToMaterial;

  // Комната в стиле Cornell box (передняя стенка отсутствует).
  const double xMin = -1.0, xMax = 1.0;
  const double yMin = 0.0, yMax = 2.0;
  const double zMin = -1.0, zMax = 1.0;

  AttachMesh(device, scene, MakeQuad(Vec3(xMin, yMin, zMin), Vec3(xMax, yMin, zMin), Vec3(xMax, yMin, zMax),
                                     Vec3(xMin, yMin, zMax)),
             matFloor, geomToMaterial);  // пол

  AttachMesh(device, scene, MakeQuad(Vec3(xMin, yMax, zMax), Vec3(xMax, yMax, zMax), Vec3(xMax, yMax, zMin),
                                     Vec3(xMin, yMax, zMin)),
             matWhite, geomToMaterial);  // потолок

  AttachMesh(device, scene, MakeQuad(Vec3(xMin, yMin, zMax), Vec3(xMin, yMax, zMax), Vec3(xMin, yMax, zMin),
                                     Vec3(xMin, yMin, zMin)),
             matRed, geomToMaterial);  // левая стена

  AttachMesh(device, scene, MakeQuad(Vec3(xMax, yMin, zMin), Vec3(xMax, yMax, zMin), Vec3(xMax, yMax, zMax),
                                     Vec3(xMax, yMin, zMax)),
             matGreen, geomToMaterial);  // правая стена

  AttachMesh(device, scene, MakeQuad(Vec3(xMin, yMin, zMin), Vec3(xMin, yMax, zMin), Vec3(xMax, yMax, zMin),
                                     Vec3(xMax, yMin, zMin)),
             matWhite, geomToMaterial);  // задняя стена

  // Внутренние объекты: один куб и одна сфера.
  AttachMesh(device, scene, MakeBox(Vec3(-0.7, 0.0, -0.2), Vec3(-0.1, 0.8, 0.4)), matMirror, geomToMaterial);
  TriangleMeshData hiPolySphere =
      MakeSphere(Vec3(0.45, 0.45, -0.1), 0.45, hiPolySphereStacks, hiPolySphereSlices);
  AttachMesh(device, scene, hiPolySphere, matGlossy, geomToMaterial);

  // Сцену нужно commit-ить ПОСЛЕ добавления всей геометрии.
  rtcCommitScene(scene);

  const Vec3 forward = Normalize(camera.target - camera.position);
  const Vec3 right = Normalize(Cross(forward, camera.up));
  const Vec3 camUp = Normalize(Cross(right, forward));
  const double aspect = static_cast<double>(width) / static_cast<double>(height);
  const double scale = std::tan((camera.fovDeg * 3.14159265358979323846 / 180.0) * 0.5);

  std::vector<Vec3> hdr(width * height, Vec3(0.0, 0.0, 0.0));

  // Основной рендер-цикл:
  // для каждого пикселя строим несколько лучей (supersampling) и усредняем результат.
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      Vec3 pixelAccum(0.0, 0.0, 0.0);
      for (int sy = 0; sy < aaSamplesY; ++sy) {
        for (int sx = 0; sx < aaSamplesX; ++sx) {
          // Стратифицированные смещения внутри пикселя (центр каждой подячейки).
          const double subX = static_cast<double>(x) + (static_cast<double>(sx) + 0.5) / aaSamplesX;
          const double subY = static_cast<double>(y) + (static_cast<double>(sy) + 0.5) / aaSamplesY;
          const double px = (2.0 * (subX / static_cast<double>(width)) - 1.0) * aspect * scale;
          const double py = (1.0 - 2.0 * (subY / static_cast<double>(height))) * scale;
          const Vec3 dir = Normalize(forward + px * right + py * camUp);
          pixelAccum += TraceRay(scene, materials, geomToMaterial, light, camera.position, dir, maxRecursionDepth, eps);
        }
      }
      hdr[y * width + x] = pixelAccum / static_cast<double>(samplesPerPixel);
    }
  }

  double maxChannel = 0.0;
  for (const Vec3 &c : hdr) {
    maxChannel = std::max(maxChannel, std::max(c.x, std::max(c.y, c.z)));
  }
  if (maxChannel <= 1e-12) {
    maxChannel = 1.0;
  }

  std::vector<Vec3> ldr(width * height);
  // Постобработка:
  // 1) нормируем по глобальному максимуму,
  // 2) применяем экспозицию, чтобы поднять средние тона,
  // 3) ограничиваем диапазон 0..1,
  // 4) применяем гамма-коррекцию для просмотра.
  for (int i = 0; i < width * height; ++i) {
    Vec3 c = hdr[i] / maxChannel;  // Нормировка по глобальному максимуму яркости.
    c = c * exposure;
    c = Clamp01(c);
    // Гамма-коррекция для корректного отображения на экране.
    c = Vec3(std::pow(c.x, 1.0 / 2.2), std::pow(c.y, 1.0 / 2.2), std::pow(c.z, 1.0 / 2.2));
    ldr[i] = Clamp01(c);
  }

  if (!SavePPM(outputFile, width, height, ldr)) {
    std::cerr << "Не удалось записать файл " << outputFile << "\n";
    rtcReleaseScene(scene);
    rtcReleaseDevice(device);
    return 1;
  }

  rtcReleaseScene(scene);
  rtcReleaseDevice(device);

  const std::size_t roomTriangles = 5 * 2;
  const std::size_t cubeTriangles = 12;
  const std::size_t sphereTriangles = hiPolySphere.indices.size();
  const std::size_t totalTriangles = roomTriangles + cubeTriangles + sphereTriangles;

  std::cout << "Антиалиасинг: " << aaSamplesX << "x" << aaSamplesY
            << " (" << samplesPerPixel << " луча на пиксель).\n";
  std::cout << "Сцена high-poly: сфера = " << sphereTriangles << " треугольников, "
            << "всего в сцене = " << totalTriangles << ".\n";
  std::cout << "Готово. Изображение сохранено в " << outputFile << " (" << width << "x" << height << ").\n";
  return 0;
}
