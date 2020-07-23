#ifndef SWAG_SCANNER_HOMECONTROLLER_H
#define SWAG_SCANNER_HOMECONTROLLER_H

#include "IController.h"

namespace controller {
    class HomeController : public IController {
    public:
        HomeController() = default;

        void run() override;
    };
}

#endif //SWAG_SCANNER_HOMECONTROLLER_H
