#include "hydraulic_sub.hpp"
#include "sub_valve_ref.hpp"

int main(){
    hydraulic_sub mysub;
    // SubValveRef mysub;

    if (mysub.init(false))
    {
        mysub.run();
    }
}
