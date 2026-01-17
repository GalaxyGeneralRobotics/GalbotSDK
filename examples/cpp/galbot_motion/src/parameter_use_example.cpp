#include <iostream>
#include <vector>
#include <string>
#include <memory>

#include "galbot_motion.hpp"

using namespace galbot::sdk::g1;

int main() {
    // 通过构造函数创建 Parameter 并设置选项
    auto p = std::make_shared<Parameter>();

    p->setBlocking(true);            // 设置是否阻塞执行
    p->setCheckCollision(false);     // 关闭碰撞检测
    p->setTimeout(5.0);              // 设置超时时间（秒）
    p->setActuate("with_chain_only");// 设置驱动模式
    p->setToolPose(false);           // 是否考虑工具位姿
    p->setReferenceFrame("base_link");

    std::cout << "--- Parameter p ---" << std::endl;
    std::cout << "blocking: " << (p->getBlocking() ? "True" : "False") << std::endl;
    std::cout << "collision check: " << (p->getCheckCollision() ? "True" : "False") << std::endl;
    std::cout << "timeout: " << p->getTimeout() << "s" << std::endl;

    return 0;
}
