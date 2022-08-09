#include "main_window.h"
#include "qgl_viewer.h"
#include "common/camera.h"

#include <QApplication>
#include <QLocale>
#include <QTranslator>

#include "getopt.h"

bool isPCDFile(const std::string &file)
{
  std::string extension(file.substr(file.find_last_of('.') + 1));
  return extension == "pcd";
}

bool isImageFile(const std::string &file)
{
  bool isImage = false;
  std::string extension(file.substr(file.find_last_of('.') + 1));

  isImage = (extension == "png") ||
            (extension == "jpg") ||
            (extension == "jpeg");

  return isImage;
}
static void printUsage()
{
  std::cout << "Usage: duna_viewer [-options] [.pcd]..." << std::endl
            << std::endl;
  std::cout << "  -h  print help" << std::endl;
  std::cout << "  -c  calibration only" << std::endl;
  std::cout << "      This option allows loading image files as well" << std::endl;
}

int main(int argc, char *argv[])
{
  QApplication a(argc, argv);

  int opt;
  bool calibration_only = false;

  QMainWindow *main_window;

  while ((opt = getopt(argc, argv, "ch")) != -1)
  {

    switch (opt)
    {
    case 'c':
      calibration_only = true;
      break;

    case 'h':
      printUsage();
      exit(0);
    }
  }

  if (calibration_only)
  {
    main_window = new (CalibrationWindow);
  }
  else
  {
    main_window = new (MainWindow);
  }

  if (optind < argc)
  {
    // Check non-opt arguments
    // TODO check first argument only (if multiple)
    while (optind < argc)
    {
      std::string arg(argv[optind]);

      if (calibration_only)
      {
        CalibrationWindow *window = reinterpret_cast<CalibrationWindow *>(main_window);
        if (isImageFile(arg))
        {
          std::cout << "Image argument\n";
          window->setImageFile(arg);
        }

        if (isPCDFile(arg))
        {
          window->setPointCloudFile(arg);
        }
      }

      else
      {
        MainWindow *window = reinterpret_cast<MainWindow *>(main_window);
        if (isPCDFile(arg))
        {
          window->setPointCloudFile(arg);
        }
      }
      optind++;
    }
  }

  main_window->show();

  return a.exec();
  delete main_window;
}
