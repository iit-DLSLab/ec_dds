#ifndef NET_STAT_HPP
#define NET_STAT_HPP

class NetworkStatistics{
public:
    NetworkStatistics();

    void run(unsigned long sequence_id, double timestamp);

    unsigned long sequence_id_prec;
    double timestamp_prec;

    double delta_timestamp;
    unsigned long  missed_packages_tot;
    unsigned long  missed_packages_curr;
};

#endif