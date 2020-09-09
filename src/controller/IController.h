#ifndef SWAG_SCANNER_ICONTROLLER_H
#define SWAG_SCANNER_ICONTROLLER_H

#include <QObject>
#include <QRunnable>

namespace controller {
    /**
     * Represents an abstract base for controllers.
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
