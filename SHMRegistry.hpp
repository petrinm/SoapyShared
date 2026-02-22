#pragma once

#include <string>
#include <vector>

namespace SHMRegistry
{

	/* List all SHMs */
	std::vector<std::string> list();

	/* Add a shared memory segment name to the registry if it doesn't already exist. */
	void add(const std::string &name);

	/* Remove a shared memory segment name from the registry. */
	void remove(const std::string &name);

};