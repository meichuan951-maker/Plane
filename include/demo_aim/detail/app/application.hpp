// 应用层入口声明：返回退出码 0=正常、1=运行错误、130=信号中断。
#pragma once
namespace aim::app {
    int run(int argc, char **argv);
}
