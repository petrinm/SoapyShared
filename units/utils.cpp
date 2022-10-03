#include "utils.hpp"

#include <boost/filesystem.hpp>



std::vector<std::string> listSHMs() {
	std::vector<std::string> shms;

#if defined(__linux__) || defined(__unix__) || defined(__APPLE__)
	/*
	 * POSIX implementation
	 */
	using namespace boost::filesystem;
	path p("/dev/shm/");
	directory_iterator end_itr;
	for (directory_iterator itr(p); itr != end_itr; ++itr)
		shms.push_back(itr->path().filename().string());

#elif defined(_WIN32)
	/*
	 * Windows implementation
	 */
	// TODO: Is this even possible without creating another SHM just to keep a list of SHMs...

#endif
	return shms;
}
