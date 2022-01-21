#include <iostream>
#include "turtlelib/rigid2d.hpp"
using namespace turtlelib;
int main()
{
    Transform2D tfab,tfbc;
    std::cout<< "Enter transform T_{a,b}:" << std::endl;
    std::cin >> tfab;
    std::cout<< "Enter transform T_{b,c}:" << std::endl;
    std::cin >> tfbc;
    std::cout << "T_{a,b}: " << tfab;
    std::cout << "T_{b,a}: " << tfab.inv();
    std::cout << "T_{b,c}: " << tfbc;
    std::cout << "T_{c,b}: " << tfbc.inv();
    Transform2D tfac = tfab*tfbc;
    std::cout << "T_{a,c}: " << tfac;
    std::cout << "T_{c,a}: " << tfac.inv();
    Vector2D vb;
    std::cout << "Enter vector v_b:" << std::endl;
    std::cin >> vb;
    std::cout << "v_bhat: " << Vector2Dnormalize(vb);
    std::cout << "v_a: " << tfab(vb);
    std::cout << "v_b: " << vb;
    std::cout << "v_c: " << tfbc.inv()(vb);
    std::cout << "Enter twist V_b: " << std::endl;
    Twist2D twb;
    std::cin >> twb;
    std::cout << "V_a: " << tfab(twb);
    std::cout << "V_b: " << twb;
    std::cout << "V_c: " << tfbc.inv()(twb);
}