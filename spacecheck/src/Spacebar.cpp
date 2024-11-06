#include <iostream>
#include <chrono>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

using namespace BT;
using namespace std::chrono_literals;

bool isKeyPressed(int key) {
    struct termios oldt, newt;
    int oldf;
    char ch;
    bool keyPressed = false;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);  
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    if (ch == key) {
        keyPressed = true;
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    return keyPressed;
}


class CheckKeyPress : public BT::SyncActionNode {
public:
    CheckKeyPress(const std::string& name) : 
        BT::SyncActionNode(name, {}) 
    {}

    BT::NodeStatus tick() override 
    {
        if (isKeyPressed(32)) { 
            std::cout << "Spacebar Status: Success." << std::endl; 
            return BT::NodeStatus::SUCCESS; 
        }
        std::cout << " Spacebar Status: Failure." << std::endl; 
        return BT::NodeStatus::FAILURE; 
    }
};

class OutputKeyPress : public BT::SyncActionNode {
public:
    OutputKeyPress(const std::string& name) : 
        BT::SyncActionNode(name, {}) 
    {}

    BT::NodeStatus tick() override {
        std::cout << " Key is pressed." << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

int main() {
    BehaviorTreeFactory factory;

    factory.registerNodeType<CheckKeyPress>("CheckKeyPress");
    factory.registerNodeType<OutputKeyPress>("OutputKeyPress");

    auto tree = factory.createTreeFromFile("/home/addienze/MagangBanyubramanta/Main/cek/spacecheck/behavior_trees/spacecheck.xml");   
    
    for (;;) {
        tree.tickRoot();  
        std::this_thread::sleep_for(std::chrono::milliseconds(150)); // Wait 1 second before next tick
    }


    return 0;
}


#
