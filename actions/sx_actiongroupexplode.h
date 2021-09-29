#ifndef SX_ACTIONGROUPEXPLODE_H
#define SX_ACTIONGROUPEXPLODE_H


#include "rs_previewactioninterface.h"

class sx_actionGroupExplode : public RS_PreviewActionInterface {
    Q_OBJECT
public:
    sx_actionGroupExplode(RS_EntityContainer& container,
                        RS_GraphicView& graphicView);

    void init(int status=0) override;

    void trigger() override;

    int size();
    int tmpNumber;
};


#endif // SX_ACTIONGROUPEXPLODE_H
