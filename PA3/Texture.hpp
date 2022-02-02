//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v) 
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        int u_img_l = u_img, u_img_r = u_img + 1;
        int v_img_l = v_img, v_img_r = v_img + 1;
        auto convert = [&](cv::Vec3b color) -> Eigen::Vector3f {
            return Eigen::Vector3f(color[0], color[1], color[2]);
        };
        auto color_lt = convert(image_data.at<cv::Vec3b>(v_img_l, u_img_r));
        auto color_rt = convert(image_data.at<cv::Vec3b>(v_img_r, u_img_r));
        auto color_lb = convert(image_data.at<cv::Vec3b>(v_img_l, u_img_l));
        auto color_rb = convert(image_data.at<cv::Vec3b>(v_img_r, u_img_l));
        auto color_top = color_lt * (v_img_r - v_img) + color_rt * (v_img - v_img_l);
        auto color_bot = color_lb * (v_img_r - v_img) + color_rb * (v_img - v_img_l);
        auto color = color_bot * (u_img_r - u_img) + color_top * (u_img - u_img_l);
        return color;
    }
};
#endif //RASTERIZER_TEXTURE_H
