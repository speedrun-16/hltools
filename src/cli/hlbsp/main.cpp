#include "../../common/log.h"
#include "../tools/bsp_tool.h"

int main(int argc, char **argv)
{
    logging::init_console();
    return tools::run_bsp_tool(argc, argv);
}
