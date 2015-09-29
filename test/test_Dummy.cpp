#include <boost/test/unit_test.hpp>
#include <sam/Dummy.hpp>

using namespace sam;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    sam::DummyClass dummy;
    dummy.welcome();
}
