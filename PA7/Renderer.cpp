//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include <atomic>
#include <thread>
#include <mutex>
#include "Scene.hpp"
#include "Renderer.hpp"


inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    std::atomic<int> m = 0;
    std::mutex mut;

    // change the spp value to change sample ammount
    int spp = 256;
    std::cout << "SPP: " << spp << "\n";
    auto deal = [&](int lx, int rx, int ly, int ry) -> void {
        for (uint32_t j = ly; j < ry; ++j) {
            for (uint32_t i = lx; i < rx; ++i) {
                // generate primary ray direction
                int index = j * scene.width + i;
                for (int k = 0; k < spp; k++){
                    float x = (2 * (i + get_random_float()) / (float)scene.width - 1) *
                            imageAspectRatio * scale;
                    float y = (1 - 2 * (j + get_random_float()) / (float)scene.height) * scale;
                    Vector3f dir = normalize(Vector3f(-x, y, 1));
                    framebuffer[index] += scene.castRay(Ray(eye_pos, dir), 0) / spp;
                }
            }
            m += rx - lx;
            {
                std::lock_guard<std::mutex> lock(mut);
                UpdateProgress(m / ((float)scene.height * scene.width));
            }
        }
    };

    int x_base = 5, y_base = 5, con = 0;
    int x_step = ceil(double(scene.width) / x_base), y_step = ceil(double(scene.height) / y_base);
    std::thread thd[x_base * y_base];
    for(int i = 0; i < x_base; i++) {
        for(int j = 0; j < y_base; j++) {
            int lx = i * x_step, rx = std::min((i + 1) * x_step, scene.width);
            int ly = j * y_step, ry = std::min((j + 1) * y_step, scene.height);
            thd[con] = std::thread(deal, lx, rx, ly, ry);
            con++;
        }
    }
    for(int i = 0; i < x_base * y_base; i++) thd[i].join();

    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
