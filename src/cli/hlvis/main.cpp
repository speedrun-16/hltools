#include "../../common/log.h"
#include "../tools/vis_tool.h"

int main(int argc, char **argv)
{
    logging::init_console();
    return tools::run_vis_tool(argc, argv);
}
