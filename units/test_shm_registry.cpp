
#include <vector>
#include <algorithm>
#include <string>

#include <cppunit/CompilerOutputter.h>
#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/extensions/HelperMacros.h>

#include "../Utils.hpp"

using namespace std;

class SHMRegistryTest: public CppUnit::TestFixture {
public:
	const string test_shm1 = "test_shm_1";
	const string test_shm2 = "test_shm_2";
	const string test_shm3 = "test_shm_3";

	void setUp()
	{
		// Clean up any existing entries
		SHMRegistry::remove(test_shm1);
		SHMRegistry::remove(test_shm2);
		SHMRegistry::remove(test_shm3);
	}

	void tearDown()
	{
		// Clean up
		SHMRegistry::remove(test_shm1);
		SHMRegistry::remove(test_shm2);
		SHMRegistry::remove(test_shm3);
	}

	/**
	 * Test that the registry list correctly reports absence of test entries.
	 * Verifies that after cleanup in setUp(), the registry does not contain
	 * the test shared memory entries. This ensures the registry state is
	 * properly initialized for each test and that list() correctly reflects
	 * the current state of registered shared memory objects.
	 */
	void testEmptyList()
	{
		vector<string> shms = SHMRegistry::list();
		// List might not be empty if other tests left entries, but we can check our entries are not there
		CPPUNIT_ASSERT(find(shms.begin(), shms.end(), test_shm1) == shms.end());
		CPPUNIT_ASSERT(find(shms.begin(), shms.end(), test_shm2) == shms.end());
	}

	/**
	 * Test adding a single entry to the registry.
	 * Verifies that add() successfully registers a shared memory object name
	 * and that the registered entry appears in the list returned by list().
	 * This is the basic positive test for the add operation.
	 */
	void testAdd()
	{
		// Add an entry
		SHMRegistry::add(test_shm1);

		// Check it's in the list
		vector<string> shms = SHMRegistry::list();
		CPPUNIT_ASSERT(find(shms.begin(), shms.end(), test_shm1) != shms.end());
	}

	/**
	 * Test adding multiple entries to the registry.
	 * Verifies that multiple shared memory objects can be registered simultaneously
	 * and that all entries are correctly tracked in the registry. This ensures
	 * the registry can handle multiple concurrent registrations without losing
	 * or corrupting entries.
	 */
	void testAddMultiple()
	{
		// Add multiple entries
		SHMRegistry::add(test_shm1);
		SHMRegistry::add(test_shm2);
		SHMRegistry::add(test_shm3);

		// Check all are in the list
		vector<string> shms = SHMRegistry::list();
		CPPUNIT_ASSERT(find(shms.begin(), shms.end(), test_shm1) != shms.end());
		CPPUNIT_ASSERT(find(shms.begin(), shms.end(), test_shm2) != shms.end());
		CPPUNIT_ASSERT(find(shms.begin(), shms.end(), test_shm3) != shms.end());
	}

	/**
	 * Test that adding duplicate entries doesn't create duplicates in the registry.
	 * Verifies that calling add() multiple times with the same name results in
	 * the entry appearing only once in the list. This ensures the registry
	 * maintains uniqueness and prevents duplicate entries from cluttering the
	 * registry or causing confusion.
	 */
	void testAddDuplicate()
	{
		// Add the same entry twice
		SHMRegistry::add(test_shm1);
		SHMRegistry::add(test_shm1);

		// Should only appear once
		vector<string> shms = SHMRegistry::list();
		size_t count = count_if(shms.begin(), shms.end(),
			[this](const string& s) { return s == test_shm1; });
		CPPUNIT_ASSERT(count == 1);
	}

	/**
	 * Test removing a single entry from the registry.
	 * Verifies that remove() successfully unregisters a shared memory object
	 * and that the entry no longer appears in the list after removal. This
	 * ensures the registry correctly tracks the lifecycle of shared memory
	 * objects and that removal operations work as expected.
	 */
	void testRemove()
	{
		// Add an entry
		SHMRegistry::add(test_shm1);

		// Verify it's there
		vector<string> shms = SHMRegistry::list();
		CPPUNIT_ASSERT(find(shms.begin(), shms.end(), test_shm1) != shms.end());

		// Remove it
		SHMRegistry::remove(test_shm1);

		// Verify it's gone
		shms = SHMRegistry::list();
		CPPUNIT_ASSERT(find(shms.begin(), shms.end(), test_shm1) == shms.end());
	}

	/**
	 * Test removing one entry from multiple entries without affecting others.
	 * Verifies that when multiple entries exist in the registry, removing one
	 * entry only affects that specific entry and leaves all other entries
	 * intact. This ensures the registry correctly isolates removal operations
	 * and doesn't accidentally remove or corrupt other entries.
	 */
	void testRemoveMultiple()
	{
		// Add multiple entries
		SHMRegistry::add(test_shm1);
		SHMRegistry::add(test_shm2);
		SHMRegistry::add(test_shm3);

		// Remove one
		SHMRegistry::remove(test_shm2);

		// Check the others are still there
		vector<string> shms = SHMRegistry::list();
		CPPUNIT_ASSERT(find(shms.begin(), shms.end(), test_shm1) != shms.end());
		CPPUNIT_ASSERT(find(shms.begin(), shms.end(), test_shm2) == shms.end());
		CPPUNIT_ASSERT(find(shms.begin(), shms.end(), test_shm3) != shms.end());
	}

	/**
	 * Test that removing a non-existent entry doesn't cause errors.
	 * Verifies that remove() handles the case of attempting to remove an entry
	 * that doesn't exist gracefully (doesn't crash or throw exceptions) and
	 * that the registry continues to function normally afterward. This is a
	 * negative test ensuring robust error handling for edge cases.
	 */
	void testRemoveNonExistent()
	{
		// Removing a non-existent entry should not crash
		SHMRegistry::remove("non_existent_shm");

		// Should still work normally
		SHMRegistry::add(test_shm1);
		vector<string> shms = SHMRegistry::list();
		CPPUNIT_ASSERT(find(shms.begin(), shms.end(), test_shm1) != shms.end());
	}

	/**
	 * Test that the registry list remains consistent across multiple calls.
	 * Verifies that calling list() multiple times returns consistent results,
	 * ensuring that the registry state is stable and that entries persist
	 * correctly between list operations. This tests the persistence and
	 * consistency of the registry state.
	 */
	void testListPersistence()
	{
		// Add an entry
		SHMRegistry::add(test_shm1);

		// Get list twice - should be consistent
		vector<string> shms1 = SHMRegistry::list();
		vector<string> shms2 = SHMRegistry::list();

		CPPUNIT_ASSERT(find(shms1.begin(), shms1.end(), test_shm1) != shms1.end());
		CPPUNIT_ASSERT(find(shms2.begin(), shms2.end(), test_shm1) != shms2.end());
	}

	CPPUNIT_TEST_SUITE(SHMRegistryTest);
	CPPUNIT_TEST(testEmptyList);
	CPPUNIT_TEST(testAdd);
	CPPUNIT_TEST(testAddMultiple);
	CPPUNIT_TEST(testAddDuplicate);
	CPPUNIT_TEST(testRemove);
	CPPUNIT_TEST(testRemoveMultiple);
	CPPUNIT_TEST(testRemoveNonExistent);
	CPPUNIT_TEST(testListPersistence);
	CPPUNIT_TEST_SUITE_END();
};

CPPUNIT_TEST_SUITE_REGISTRATION(SHMRegistryTest);

int main(int argc, char *argv[]) {
	CppUnit::TextUi::TestRunner runner;
	CppUnit::TestFactoryRegistry &registry = CppUnit::TestFactoryRegistry::getRegistry();
	runner.addTest(registry.makeTest());
	runner.run();
	return 0;
}

