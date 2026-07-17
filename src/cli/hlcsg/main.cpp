#include "../../common/log.h"
#include "../tools/csg_tool.h"

int main(int argc, char **argv)
{
    logging::init_console();
    return tools::run_csg_tool(argc, argv);
}
