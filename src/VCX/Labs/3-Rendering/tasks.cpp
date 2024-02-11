#include "Labs/3-Rendering/tasks.h"

namespace VCX::Labs::Rendering {

    glm::vec4 GetTexture(Engine::Texture2D<Engine::Formats::RGBA8> const& texture, glm::vec2 const& uvCoord) {
        if (texture.GetSizeX() == 1 || texture.GetSizeY() == 1) return texture.At(0, 0);
        glm::vec2 uv = glm::fract(uvCoord);
        uv.x = uv.x * texture.GetSizeX() - .5f;
        uv.y = uv.y * texture.GetSizeY() - .5f;
        std::size_t xmin = std::size_t(glm::floor(uv.x) + texture.GetSizeX()) % texture.GetSizeX();
        std::size_t ymin = std::size_t(glm::floor(uv.y) + texture.GetSizeY()) % texture.GetSizeY();
        std::size_t xmax = (xmin + 1) % texture.GetSizeX();
        std::size_t ymax = (ymin + 1) % texture.GetSizeY();
        float       xfrac = glm::fract(uv.x), yfrac = glm::fract(uv.y);
        return glm::mix(glm::mix(texture.At(xmin, ymin), texture.At(xmin, ymax), yfrac), glm::mix(texture.At(xmax, ymin), texture.At(xmax, ymax), yfrac), xfrac);
    }

    glm::vec4 GetAlbedo(Engine::Material const& material, glm::vec2 const& uvCoord) {
        glm::vec4 albedo = GetTexture(material.Albedo, uvCoord);
        glm::vec3 diffuseColor = albedo;
        return glm::vec4(glm::pow(diffuseColor, glm::vec3(2.2)), albedo.w);
    }

    /******************* 1. Ray-triangle intersection *****************/
    bool IntersectTriangle(Intersection& output, Ray const& ray, glm::vec3 const& p1, glm::vec3 const& p2, glm::vec3 const& p3) {
        // your code here
        glm::mat3 theta_uvt = glm::mat3(p2 - p1, p3 - p1, -ray.Direction);
        float epsilon = 1e-6;
        if (glm::abs(glm::determinant(theta_uvt)) < epsilon) { return false; }
        glm::vec3 uvt = glm::inverse(theta_uvt) * (ray.Origin - p1);
        if (uvt.x < 0 || uvt.x > 1 || uvt.y < 0 || uvt.y > 1 || uvt.z < 0 || (uvt.x + uvt.y) > 1) { return false; }
        output.u = uvt.x;
        output.v = uvt.y;
        output.t = uvt.z * glm::sqrt(glm::dot(ray.Direction, ray.Direction));
        return true;
    }

    glm::vec3 RayTrace(const RayIntersector& intersector, Ray ray, int maxDepth, bool enableShadow) {
        glm::vec3 color(0.0f);
        glm::vec3 weight(1.0f);

        for (int depth = 0; depth < maxDepth; depth++) {
            auto rayHit = intersector.IntersectRay(ray);
            if (!rayHit.IntersectState) return color;
            const glm::vec3 pos = rayHit.IntersectPosition;
            const glm::vec3 n = rayHit.IntersectNormal;
            const glm::vec3 kd = rayHit.IntersectAlbedo;
            const glm::vec3 ks = rayHit.IntersectMetaSpec;
            const float     alpha = rayHit.IntersectAlbedo.w;
            const float     shininess = rayHit.IntersectMetaSpec.w * 256;

            glm::vec3 result(0.0f);
            /******************* 2. Whitted-style ray tracing *****************/
            // your code here
            glm::vec3 viewDir = -glm::normalize(ray.Direction);
            glm::vec3 normal = glm::normalize(n);

            for (const Engine::Light& light : intersector.InternalScene->Lights) {
                glm::vec3 l;
                float     attenuation;
                /******************* 3. Shadow ray *****************/
                if (light.Type == Engine::LightType::Point) {
                    l = light.Position - pos;
                    attenuation = 1.0f / glm::dot(l, l);
                    if (enableShadow) {
                        // your code here
                        glm::vec3 new_pos = pos;
                        while (true) {
                            auto new_hit = intersector.IntersectRay(Ray(new_pos, l));
                            if (!new_hit.IntersectState) { break; }
                            else {
                                glm::vec3 hit_pos = new_hit.IntersectPosition;
                                if (dot(l, l) <= dot(hit_pos - pos, l)) { break; }
                                if (new_hit.IntersectAlbedo.w < 0.2) {
                                    new_pos = new_hit.IntersectPosition;
                                }
                                else {
                                    attenuation *= (1 - new_hit.IntersectAlbedo.w);
                                    break;
                                }
                            }
                        }
                    }
                }
                else if (light.Type == Engine::LightType::Directional) {
                    l = light.Direction;
                    attenuation = 1.0f;
                    if (enableShadow) {
                        // your code here
                        glm::vec3 new_pos = pos;
                        while (true) {
                            auto new_hit = intersector.IntersectRay(Ray(new_pos, l));
                            if (!new_hit.IntersectState) { break; }
                            if (new_hit.IntersectAlbedo.w < 0.2) {
                                new_pos = new_hit.IntersectPosition;
                            }
                            else {
                                attenuation *= (1 - new_hit.IntersectAlbedo.w);
                                break;
                            }
                        }

                    }
                }

                /******************* 2. Whitted-style ray tracing *****************/
                // your code here
                l = glm::normalize(l);
                result = (kd * glm::max(0.0f, glm::dot(l, normal)) +
                    ks * pow(glm::max(0.0f, glm::dot(n, glm::normalize(l + viewDir))), shininess))
                    * light.Intensity * attenuation;
            }

            if (alpha < 0.9) {
                // refraction
                // accumulate color
                glm::vec3 R = alpha * glm::vec3(1.0f);
                color += weight * R * result;
                weight *= glm::vec3(1.0f) - R;

                // generate new ray
                ray = Ray(pos, ray.Direction);
            }
            else {
                // reflection
                // accumulate color
                glm::vec3 R = ks * glm::vec3(0.5f);
                color += weight * (glm::vec3(1.0f) - R) * result;
                weight *= R;

                // generate new ray
                glm::vec3 out_dir = ray.Direction - glm::vec3(2.0f) * n * glm::dot(n, ray.Direction);
                ray = Ray(pos, out_dir);
            }
        }

        return color;
    }
} // namespace VCX::Labs::Rendering