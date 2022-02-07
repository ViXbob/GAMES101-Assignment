#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        std::vector<Object*> leftshapes, rightshapes;
        if(splitMethod == SplitMethod::NAIVE) {
            Bounds3 centroidBounds;
            for (int i = 0; i < objects.size(); ++i)
                centroidBounds =
                    Union(centroidBounds, objects[i]->getBounds().Centroid());
            int dim = centroidBounds.maxExtent();
            switch (dim) {
            case 0:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().x <
                        f2->getBounds().Centroid().x;
                });
                break;
            case 1:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().y <
                        f2->getBounds().Centroid().y;
                });
                break;
            case 2:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().z <
                        f2->getBounds().Centroid().z;
                });
                break;
            }

            auto beginning = objects.begin();
            auto middling = objects.begin() + (objects.size() / 2);
            auto ending = objects.end();

            leftshapes = std::vector<Object*>(beginning, middling);
            rightshapes = std::vector<Object*>(middling, ending);
        } else if(splitMethod == SplitMethod::SAH) {
            const int B = 32;
            float MinCost = std::numeric_limits<float>::infinity();
            int MinId = 0, MinDim = 0;
            auto getValue = [](Vector3f v, int index) {
                if(index == 0) return v.x;
                if(index == 1) return v.y;
                if(index == 2) return v.z;
            };
            for(int dim = 0; dim < 3; dim++) {
                std::sort(objects.begin(), objects.end(), [&dim, &getValue](auto f1, auto f2) {
                    // return f1 -> getBounds().Centroid()[dim] < f2 -> getBounds().Centroid()[dim];
                    return getValue(f1->getBounds().Centroid(), dim) < getValue(f2->getBounds().Centroid(), dim);
                });
                float l = getValue(bounds.pMin, dim), len = (getValue(bounds.pMax, dim) - getValue(bounds.pMin, dim));
                std::vector<Bounds3> suf(B + 5);
                for(int i = B - 1, p = objects.size() - 1; i >= 1; i--) {
                    float barrier = l + len / B * i;
                    suf[i] = suf[i + 1];
                    while(p >= 0 && getValue(objects[p]->getBounds().Centroid(), dim) >= barrier) {
                        suf[i] = Union(suf[i], objects[p]->getBounds());
                        p--;
                    }
                }
                Bounds3 pre;
                for(int i = 1, p = 0; i < B; i++) {
                    float barrier = l + len / B * i;
                    while(p < objects.size() && getValue(objects[p]->getBounds().Centroid(), dim) < barrier) {
                        pre = Union(pre, objects[p]->getBounds());
                        p++;
                    }
                    float cost = p * pre.SurfaceArea() + (objects.size() - p - 1) * suf[i].SurfaceArea();
                    if(cost < MinCost) {
                        MinCost = cost;
                        MinId = p;
                        MinDim = dim;
                    }
                }

            }
            auto beginning = objects.begin();
            auto middling = objects.begin() + MinId;
            auto ending = objects.end();
            std::sort(objects.begin(), objects.end(), [&MinDim, &getValue](auto f1, auto f2) {
                // return f1 -> getBounds().Centroid()[dim] < f2 -> getBounds().Centroid()[dim];
                return getValue(f1->getBounds().Centroid(), MinDim) < getValue(f2->getBounds().Centroid(), MinDim);
            });
            leftshapes = std::vector<Object*>(beginning, middling);
            rightshapes = std::vector<Object*>(middling, ending);
        }
        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    // auto [flag, t_enter] = root -> bounds.IntersectWithTime(ray, ray.direction_inv, ray.dirIsNeg);
    // if(flag) isect = BVHAccel::getIntersection(root, ray);
    getIntersection(root, ray, isect);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    Intersection inter;
    if(!node) return inter;
    if(node -> object) {
        inter = node -> object -> getIntersection(ray);
        return inter;
    }

    auto [flag_l, t_enter_l] = node -> left -> bounds.IntersectWithTime(ray, ray.direction_inv, ray.dirIsNeg);
    auto [flag_r, t_enter_r] = node -> right -> bounds.IntersectWithTime(ray, ray.direction_inv, ray.dirIsNeg);
    if(t_enter_l < t_enter_r) {
        if(flag_l) inter = getIntersection(node -> left, ray);
        if(flag_r && t_enter_r < inter.distance) {
            Intersection r = getIntersection(node -> right, ray);
            if(inter.distance > r.distance) inter = r;
        }
    } else {
        if(flag_r) inter = getIntersection(node -> right, ray);
        if(flag_l && t_enter_l < inter.distance) {
            Intersection l = getIntersection(node -> left, ray);
            if(inter.distance > l.distance) inter = l;
        }
    }
    return inter;
}

void BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray, Intersection& optimal) const {
    if(!node) return;
    if(node -> object) {
        auto inter = node -> object -> getIntersection(ray);
        if(inter.distance < optimal.distance) optimal = inter;
        return;
    }
    BVHBuildNode *l = node -> left, *r = node -> right;
    auto [flag_l, t_enter_l] = l -> bounds.IntersectWithTime(ray, ray.direction_inv, ray.dirIsNeg);
    auto [flag_r, t_enter_r] = r -> bounds.IntersectWithTime(ray, ray.direction_inv, ray.dirIsNeg);
    if(t_enter_l > t_enter_r) {
        std::swap(flag_l, flag_r);
        std::swap(t_enter_l, t_enter_r);
        std::swap(l, r);
    }
    if(flag_l && t_enter_l < optimal.distance) getIntersection(l, ray, optimal);
    if(flag_r && t_enter_r < optimal.distance) getIntersection(r, ray, optimal); 
}

// Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
// {
//     // TODO Traverse the BVH to find intersection
//     Intersection inter;
//     if(!node) return inter;
//     std::array<int, 3> dirIsNeg;
//     for(int i = 0; i < 3; i++) dirIsNeg[i] = int(ray.direction[i] > 0);
//     bool flag = node -> bounds.IntersectP(ray, ray.direction_inv, dirIsNeg);
//     if(node -> object) {
//         if(!flag) {
//             // auto tmp = node -> object -> getIntersection(ray);
//             // if(tmp.happened) {
//             //     std::cout << tmp.coords << std::endl;
//             //     std::cout << node -> object -> getBounds().pMin << std::endl;
//             //     std::cout << node -> object -> getBounds().pMax << std::endl;
//             //     std::cout << tmp.happened << std::endl;
//             // }
//             return inter;
//         }
//         inter = node -> object -> getIntersection(ray);
//         return inter;
//     }

//     if(flag) {
//         Intersection l = getIntersection(node -> left, ray);
//         Intersection r = getIntersection(node -> right, ray);
//         return l.distance < r.distance ? l : r;
//     }
//     return inter;
// }