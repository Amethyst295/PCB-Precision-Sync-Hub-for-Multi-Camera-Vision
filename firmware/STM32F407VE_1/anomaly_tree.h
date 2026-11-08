#ifndef ANOMALY_TREE_H
#define ANOMALY_TREE_H
#define ANOM_NFEAT 9
typedef enum {
    FAULT_NORMAL = 0,
    FAULT_CAM_DISCONNECTED = 1,
    FAULT_INTERMITTENT_CABLE = 2,
    FAULT_LATENCY_DRIFT = 3,
    FAULT_DOUBLE_TRIGGER_EMI = 4,
    FAULT_PTP_MASTER_LOSS = 5,
    FAULT_NET_CONGESTION = 6,
    FAULT_OSCILLATOR_FAULT = 7
} sync_fault_t;
sync_fault_t anomaly_classify(const float x[ANOM_NFEAT]);
#endif
