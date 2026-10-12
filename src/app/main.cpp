// 程序入口：把命令行交给应用层；业务逻辑从 app/application.cpp 的 run() 开始阅读。
#include "application.hpp"

int main(int argc, char **argv) {
    return aim::app::run(argc, argv); 
}
