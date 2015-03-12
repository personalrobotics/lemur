
namespace ompl_multiset
{
namespace util
{

std::string sf(const char * fmt, ...);

bool startswith(std::string s, std::string prefix);

std::string sha1(std::string in);

// volume of an n-ball
double volume_n_ball(unsigned int n);

std::string double_to_text(double in);

} // namespace util
} // namespace ompl_multiset
