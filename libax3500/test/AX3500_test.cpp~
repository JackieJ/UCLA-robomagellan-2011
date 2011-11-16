#include "gtest/gtest.h"

// ick
#define private public
#define protected public

#include "AX3500.h"

namespace {

class SampleTests : public ::testing::Test
{
protected:
	virtual void SetUp()
	{
		ax3500 = new AX3500();
	}

	AX3500 *ax3500;
};

TEST_F(SampleTests, fromHex32)
{
	EXPECT_EQ(-7986091, (int)ax3500->fromHex32("862455");
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

