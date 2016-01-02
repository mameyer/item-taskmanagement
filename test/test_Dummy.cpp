#include <boost/test/unit_test.hpp>
#include <taskmanagement/Dummy.hpp>

using namespace taskmanagement;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    taskmanagement::DummyClass dummy;
    dummy.welcome();
}
