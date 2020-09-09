#ifndef SWAG_SCANNER_ICONTROLLER_H
#define SWAG_SCANNER_ICONTROLLER_H

#include <QObject>
#include <QRunnable>

namespace controller {
    /**
     * Represents an abstract base for controllers.
     * Have to inherit from QObject and QRunnable here because Qt requires these
     * to be at the base class. SMH.
     */
    class IController : public QObject, public QRunnable  {
    public:
        /**
         * Run the controller
         */
        virtual void run();

        virtual ~IController() {}
    };
}

#endif //SWAG_SCANNER_ICONTROLLER_H
