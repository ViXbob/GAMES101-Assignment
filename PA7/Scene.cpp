//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here

    Intersection x = intersect(ray);
    if(!x.happened) return Vector3f(0.0, 0.0, 0.0);
    if(x.m->hasEmission()) {
        if(depth == 0) return x.emit;
        else return Vector3f(0.0, 0.0, 0.0);
    }
    // std::cout << "Do not emit" << std::endl;
    Vector3f wo = -normalize(ray.direction);
    Vector3f normal = x.normal.normalized();
    Vector3f p = x.coords;

    Vector3f L_dir;
    {
        float pdf_light = 1.0;
        Intersection x_prime;
        sampleLight(x_prime, pdf_light);
        Vector3f wi = (x_prime.coords - x.coords).normalized();
        Vector3f light_normal = x_prime.normal.normalized();
        Vector3f light_p = x_prime.coords;
        Ray light_ray(p, wi);
        Intersection mid = intersect(light_ray);
        if((mid.coords - x_prime.coords).norm() < EPSILON && mid.happened) {
            float dist2 = dotProduct((light_p - p), (light_p - p));
            // if(dist2 > EPSILON && fabs(pdf_light) > EPSILON) {
                // std::cout << "x_prime.emit" << std::endl;
                // std::cout << x_prime.emit << std::endl;
                // std::cout << x.m->eval(wi, wo, x.normal) << std::endl;
                // std::cout << x.normal << std::endl;
                // std::cout << x_prime.normal << std::endl;
                // std::cout << dotProduct(wi, x.normal) << ", " << dotProduct(-wi, x_prime.normal) << ", " << dist2 * pdf_light << std::endl;
                // std::cout << dist2 << ", " << pdf_light << std::endl;
            L_dir = x_prime.emit * x.m->eval(wi, wo, normal) * dotProduct(wi, normal) * dotProduct(-wi, light_normal) / (dist2 * pdf_light);
                // std::cout << "L_dir : " << L_dir << std::endl;
            // }
        }
    }

    Vector3f L_indir;
    {
        if(get_random_float() < RussianRoulette) {
            Vector3f wi = x.m->sample(wo, x.normal).normalized();
            Ray light_ray = Ray(p, wi);
            // mid = intersect(light_ray);
            // if(mid.happened) {
            //     if(!mid.m->hasEmission()) {
            //         L_indir = castRay(light_ray, depth + 1) * x.m->eval(wi, wo, x.normal) * dotProduct(wi, x.normal) / 
            //                     (x.m->pdf(wo, wi, x.normal) * RussianRoulette);
            //     }
            // }
            L_indir = castRay(light_ray, depth + 1) * x.m->eval(wi, wo, normal) * dotProduct(wi, normal) / 
                        (x.m->pdf(wi, wo, normal) * RussianRoulette);
        }
    }

    return L_dir + L_indir;
}