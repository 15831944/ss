#include "sx_rectanglearray.h"

sx_rectangleArray::sx_rectangleArray(RS_EntityContainer &container, RS_GraphicView &graphicView
                                       ,RS_ActionInterface* action_select,
                                       std::initializer_list<RS2::EntityType> const& entityTypeList)
    :RS_PreviewActionInterface("SmallSize Selected", container, graphicView)
    ,entityTypeList(entityTypeList)
    ,en(nullptr)
    ,actionSelect(action_select)
{
    array = new sx_rectArray(container,graphicView);
    array->setWindowModality(Qt::ApplicationModal);
    array->show();
    connect(array,SIGNAL(rectArrayDataSignal(int,int,bool,bool,double,double,bool,bool,bool,bool,double,double)),
            this,SLOT(arrayDataSlot(int,int,bool,bool,double,double,bool,bool,bool,bool,double,double)));
    connect(array,SIGNAL(closeWindowSignal(bool)),this,SLOT(receiveCloseSlot(bool)));
}


void sx_rectangleArray::addNewEntities(std::vector<RS_Entity*>& addList)
{
//    LC_UndoSection undo( document, handleUndo);

    for (RS_Entity* e: addList) {
        if (e) {
            container->addEntity(e);
//            undo.addUndoable(e);
        }
    }

    container->calculateBorders();

    if (graphicView) {
        graphicView->redraw(RS2::RedrawDrawing);
    }
}



sx_rectangleArray::~sx_rectangleArray()
{

}

void sx_rectangleArray::init(int status) {
    RS_ActionInterface::init(status);
    trigger();

}

void sx_rectangleArray::trigger() {

    RS_DIALOGFACTORY->updateSelectionWidget(container->countSelected(),container->totalSelectedLength());

}

void sx_rectangleArray::arrayDataSlot(int line1,int line2,bool radio1,bool radio2,double line3,double line4,
                                      bool radio3,bool radio4,bool radio5,bool radio6,double line5,double line6)
{
    std::vector<RS_Entity*> addList;

    if (graphic) {     //事先定义成一个群组
        RS_BlockList* blockList = graphic->getBlockList();
        if (blockList) {
            RS_BlockData d =
                    RS_DIALOGFACTORY->requestNewGroupDialog(blockList);
            if (!d.name.isEmpty()) {
                RS_Creation creation(container, graphicView);
                creation.createGroup(&d, {10,10}, true);
                RS_InsertData id(
                            d.name,
                {10,10},
                            RS_Vector(1.0,1.0),
                            0.0,
                            1, 1, RS_Vector(0.0,0.0)
                            );
                creation.createInsert(&id)->setSelected(true);
            }

        }
    }

    if(radio2 == true)   //选择了间距
    {

        for (int row=1;row < line2;row++)  //先行
        {
            for(auto e: *container)
            {
                if (e && e->isSelected())
                {
                    if(radio6 == true)
                    {
                        RS_Entity* ec = e->clone();
                        double sizeEntity = e->getSize().x + line3;
                        RS_Vector vector(sizeEntity,0);
                        ec->move(vector*row);
                        addList.push_back(ec);
                    }
                    else if(radio5 == true)
                    {
                        RS_Entity* ec = e->clone();
                        double sizeEntity = e->getSize().x + line3;
                        RS_Vector vector(-sizeEntity,0);
                        ec->move(vector*row);
                        addList.push_back(ec);
                    }
                }
            }
        }
        addNewEntities(addList);

        addList.clear();
        for (int column=1;column < line1;column++)
        {
            for(auto e: *container)
            {
                if (e && e->isSelected())
                {
                    if(radio3 == true)
                    {
                        RS_Entity* ec = e->clone();
                        double sizeEntity = e->getSize().y + line4;
                        RS_Vector vector(0,sizeEntity);
                        ec->move(vector*column);
                        addList.push_back(ec);
                    }
                    else if(radio4 == true)
                    {
                        RS_Entity* ec = e->clone();
                        double sizeEntity = e->getSize().y + line4;
                        RS_Vector vector(0,-sizeEntity);
                        ec->move(vector*column);
                        addList.push_back(ec);
                    }
                }
            }
        }
        addNewEntities(addList);
    }

    else if(radio1 == true)   //选择了偏移
    {
        for (int row=1;row < line2;row++)  //先行
        {
            for(auto e: *container)
            {
                if (e && e->isSelected())
                {
                    if(radio6 == true)
                    {
                        RS_Entity* ec = e->clone();
                        double sizeEntity = line6;
                        RS_Vector vector(sizeEntity,0);
                        ec->move(vector*row);
                        addList.push_back(ec);
                    }
                    else if(radio5 == true)
                    {
                        RS_Entity* ec = e->clone();
                        double sizeEntity = line6;
                        RS_Vector vector(-sizeEntity,0);
                        ec->move(vector*row);
                        addList.push_back(ec);
                    }
                }
            }
        }
        addNewEntities(addList);

        addList.clear();
        for (int column=1;column < line1;column++)
        {
            for(auto e: *container)
            {
                if (e && e->isSelected())
                {
                    if(radio3 == true)
                    {
                        RS_Entity* ec = e->clone();
                        double sizeEntity = line5;
                        RS_Vector vector(0,sizeEntity);
                        ec->move(vector*column);
                        addList.push_back(ec);
                    }
                    else if(radio4 == true)
                    {
                        RS_Entity* ec = e->clone();
                        double sizeEntity = line5;
                        RS_Vector vector(0,-sizeEntity);
                        ec->move(vector*column);
                        addList.push_back(ec);
                    }
                }
            }
        }
        addNewEntities(addList);
    }

    for(auto ec: container->getEntityList()){  //打散群组
        if(ec->isSelected())
        {
            if(ec->rtti() == RS2::EntityInsert)
            {
                RS_ActionBlocksExplode* blockexplode  = new RS_ActionBlocksExplode(*container, *graphicView);
            }
        }
    }
    finish(false);
}

void sx_rectangleArray::receiveCloseSlot(bool isClose)
{
    if(isClose == true)
    {
        finish(false);
        array->close();
    }
}


