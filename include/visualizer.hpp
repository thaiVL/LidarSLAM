#include <vector>

void visualizeScan(const std::vector<std::pair<double, double>>& points);
void replayScans(const RobotState& state, bool accumulate = false, double speed = 1.0);
