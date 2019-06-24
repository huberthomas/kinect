#include <KinectHandler.h>
#include <iostream>
#include <string>

using namespace std;

/**
 * @brief Display help information.
 */
void help()
{
    ostringstream out;
    out << "Kinect Image Capturer" << endl;
    out << endl;
    out << "[-h] Help information." << endl;
    out << "[-o outDir] Output file directory. Default is 'results' in the application directory." << endl;
    out << endl;
    out << "During execution the following key events are activated:" << endl;
    out << endl;
    out << "ESC Close application." << endl;
    out << "SPACE Save a single frame" << endl;
    out << "d Switch depth modes." << endl;
    out << "s Start/stop frame saving." << endl;
    out << "a En-/disable auto exposure." << endl;
    out << "w En-/disable white balance." << endl;
    out << "r En-/disable raw color." << endl;
    out << endl;
    cout << out.str() << endl;
}

/**
 * @brief Main entry.
 * @param argc Argument counter.
 * @param argv Argument values.
 * @return Exit code.
 */
int main(int argc, char **argv) {
    try
    {
        string outputDir{""};
        if(argc > 1) {
            for(int i = 1; i < argc; i++) {
                if(strcmp(argv[i], "-h") == 0) {
                    help();
                }
                if(strcmp(argv[i], "-o") == 0) {
                    i++;

                    if(i < argc) {
                        if(argv[i][0] == '-') {
                            throw runtime_error("-o needs an argument.");
                            break;
                        }
                        outputDir = argv[i];
                    } else {
                        throw runtime_error("-o needs an argument.");
                        break;
                    }
                }
            }
        }

        KinectHandler handler;

        if(outputDir.size() > 0) {
            handler.setOutputDir(outputDir);
        }

        handler.startCapturing();

    } catch(exception &e) {
        help();
        cerr << e.what() << endl;
    }

    return 0;
}
