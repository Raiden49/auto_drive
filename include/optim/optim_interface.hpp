/*
 * @Author: Raiden49 
 * @Date: 2024-06-26 10:24:51 
 * @Last Modified by: Raiden49
 * @Last Modified time: 2024-08-15 16:02:25
 */
#ifndef OPTIM_INTERFACE_HPP_
#define OPTIM_INTERFACE_HPP_

#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include <iostream>
#include <time.h>
#include <memory>

#include "m_common.hpp"

namespace optimization 
{
/**
 * @brief 优化父类，没有具体实现，纯接口类
 */
using Point = auto_drive::PathPoint;
class OptimInterface {
    public:
        OptimInterface(const std::vector<Point>& path) {
            path_ptr_ = std::make_shared<std::vector<Point>>(path);
        };
        virtual ~OptimInterface() {};
        /**
         * @brief 优化主运行函数，不提供父类实现
         * 
         * @return std::vector<std::array<double, 2>> 优化后的路径
         */
        virtual std::vector<Point> Process() = 0;
    public:
        std::shared_ptr<const std::vector<Point>> path_ptr_;
};
}

#endif // OPTIM_INTERFACE_HPP_