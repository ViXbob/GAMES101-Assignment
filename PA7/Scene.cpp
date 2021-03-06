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
        // epsilon should not be too small, 
        // otherwise it will lead to misjudgment that there is occlusion between the light source and the object
        // epsilon is set 0.01
        if((mid.coords - light_p).norm() < EPSILON) {
            float dist2 = dotProduct((light_p - p), (light_p - p));
            L_dir = x_prime.emit * x.m->eval(wi, wo, normal) * dotProduct(wi, normal) * dotProduct(-wi, light_normal) / (dist2 * pdf_light);
        }
    }

    Vector3f L_indir;
    {
        if(get_random_float() < RussianRoulette) {
            Vector3f wi = x.m->sample(wo, x.normal).normalized();
            Ray light_ray = Ray(p, wi);
            L_indir = castRay(light_ray, depth + 1) * x.m->eval(wi, wo, normal) * dotProduct(wi, normal) / 
                        (x.m->pdf(wi, wo, normal) * RussianRoulette);
        }
    }

    return L_dir + L_indir;
}