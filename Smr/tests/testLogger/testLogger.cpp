/**
 *  This program is a test of the Logging capabilities of the SMR library
 */


#include "Smr.h"


void printMessages()
{
    LOG_TRACE(logger, "printMessages()");
    LOG_DEBUG(logger, "This is a DEBUG message");
    LOG_INFO(logger, "This is a INFO message");
    LOG_WARN(logger, "This is a WARN message");
    LOG_ERROR(logger, "This is a ERROR message");
    LOG_FATAL(logger, "This is a FATAL message");
}


int
main()
{

    Smr::initSmr(false);

    logger.setLogLevel(TRACE_LOG_LEVEL);
    cout << "*** calling printMessages() with TRACE set: ***" << endl;
    printMessages();

    logger.setLogLevel(DEBUG_LOG_LEVEL);
    cout << "\n*** calling printMessages() with DEBUG set: ***" << endl;
    printMessages();

    logger.setLogLevel(INFO_LOG_LEVEL);
    cout << "\n*** calling printMessages() with INFO set: ***" << endl;
    printMessages();

    logger.setLogLevel(WARN_LOG_LEVEL);
    cout << "\n*** calling printMessages() with WARN set: ***" << endl;
    printMessages();

    logger.setLogLevel(ERROR_LOG_LEVEL);
    cout << "\n*** calling printMessages() with ERROR set: ***" << endl;
    printMessages();

    logger.setLogLevel(FATAL_LOG_LEVEL);
    cout << "\n*** calling printMessages() with FATAL set: ***" << endl;
    printMessages();

    return 0;
}
