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
    out << "[-s] Synchronize RGB with depth files and store results to file. Needs -o outDir.";
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
        bool syncFiles{false};

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
                if(strcmp(argv[i], "-s") == 0) {
                    syncFiles = true;
                }
            }
        }

        KinectHandler handler;

        if(outputDir.size() > 0) {
            handler.setOutputDir(outputDir);
        }

        if(syncFiles) {
            if(outputDir.size() > 0){
                string rgbDir = outputDir + "/rgb/";
                string depthDir = outputDir + "/depth/";
                handler.associateFiles(rgbDir, depthDir, outputDir);

                cout << "Synchronization successfull finished." << endl;
                return 0;
            } else {
                throw runtime_error("-s needs an output directory.");
            }
        }

        handler.startCapturing();

    } catch(exception &e) {
        help();
        cerr << e.what() << endl;
    }

    return 0;
}
