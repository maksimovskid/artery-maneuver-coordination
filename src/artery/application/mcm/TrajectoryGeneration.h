#ifndef TRAJECTORYGENERATION_H_
#define TRAJECTORYGENERATION_H_

#include <vector>

namespace artery
{
namespace mcm
{

std::vector<float> makeShiftedLaneCx(const std::vector<float>& cx);

} // namespace mcm
} // namespace artery

#endif /* TRAJECTORYGENERATION_H_ */
