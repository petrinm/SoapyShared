
#include "Utils.hpp"

#include <iostream>


#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>



using namespace std;
using namespace boost::interprocess;


namespace SHMRegistry {


const char *shm_name = "soapy_registry";
const size_t max_name_len = 64;
const size_t registry_size = 16;


std::vector<std::string> list()
{
	std::vector<std::string> shms;
	try
	{
		shared_memory_object registry_shm(open_only, shm_name, read_write);
		mapped_region mapped(registry_shm, read_write, 0, registry_size * max_name_len);
		const char *name_array = static_cast<const char *>(mapped.get_address());

		for (size_t i = 0; i < registry_size; i++)
        {
			const char *list_name = &name_array[max_name_len * i];
			if (list_name[0] != '\0')
				shms.push_back(list_name);
		}
	}
	catch (boost::interprocess::interprocess_exception &e)
	{
		cerr << "SHMRegistry error: " << e.what() << endl;
	}

	return shms;
}


void add(const std::string &name)
{
	try
	{
		shared_memory_object registry_shm(open_or_create, shm_name, read_write);
		registry_shm.truncate(registry_size * max_name_len);
		mapped_region mapped(registry_shm, read_write, 0, registry_size * max_name_len);
		char *name_array = static_cast<char *>(mapped.get_address());

		const size_t invalid_index = ~0;
		size_t empty = invalid_index;
        for (size_t i = 0; i < registry_size; i++)
        {
			char *list_name = &name_array[max_name_len * i];
			if (list_name[0] == '\0' && empty == invalid_index)
				empty = i; // Note the first empty slot
			if (memcmp(list_name, name.c_str(), name.size()) == 0)
				return; // Already listed
		}

		if (empty != invalid_index) {
			char *list_name = &name_array[max_name_len * empty];
			memcpy(list_name, name.c_str(), name.size());
		}

	}
	catch (boost::interprocess::interprocess_exception &e)
	{
		cerr << "SHMRegistry error: " << e.what() << endl;
	}
}


void remove(const std::string &name)
{
	try
	{
		shared_memory_object registry_shm(open_or_create, shm_name, read_write);
        mapped_region mapped(registry_shm, read_write, 0, registry_size * max_name_len);
        char *name_array = static_cast<char *>(mapped.get_address());

        for (size_t i = 0; i < registry_size; i++)
        {
			char *list_name = &name_array[max_name_len * i];
			if (memcmp(list_name, name.c_str(), name.size()) == 0)
			{
				memset(list_name, 0, max_name_len);
				return;
			}
		}
	}
	catch (boost::interprocess::interprocess_exception &e)
	{
		cerr << "SHMRegistry error: " << e.what() << endl;
	}
}


}; // namespace SHMRegistry