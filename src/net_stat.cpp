#include "net_stat.hpp"
#include <cmath>

NetworkStatistics::NetworkStatistics()
:   sequence_id_prec(0),
    timestamp_prec(0.0),
    delta_timestamp(0.0),
    missed_packages_tot(0),
    missed_packages_curr(0)
{}

void NetworkStatistics::run(unsigned long int sequence_id, double timestamp){
    if(sequence_id !=sequence_id_prec){ // if the sequence_id are the same, the package is not changed
        missed_packages_curr = sequence_id - sequence_id_prec -1; // the input sequence_id is the precedent one +1 in case there is no missed package
        missed_packages_tot+=missed_packages_curr;
        sequence_id_prec = sequence_id;
    }
    // check anyway the timestamp (it could happen that the sequence id is not updated by the publisher. The timestamp should be always updated...)
    delta_timestamp = (timestamp - timestamp_prec)/1'000'000; //hp: timestamp in nanoseconds, expressing here in ms
    timestamp_prec = timestamp;
}