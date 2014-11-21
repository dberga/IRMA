
#include "IRMA.hpp"



void IRMA::draw3D(){



            if ((!this->window3D->DrawGLScene()) || this->window3D->getKeyStatus(VK_ESCAPE))	// Active?  Was There A Quit Received?
			{
				//quit
				std::cout << "error" << std::endl;
			}
			else
			{

			    //3d pos for each polygon
			    for(int m=0; m<_markers.size(); m++){


                    _markers[m]->polygon->setXTrans(_markers[m]->_eucl3d_ps.at<double>(0,0));
                    _markers[m]->polygon->setYTrans(_markers[m]->_eucl3d_ps.at<double>(0,1));
                    _markers[m]->polygon->setZTrans(_markers[m]->_eucl3d_ps.at<double>(0,2));

                     _markers[m]->polygon->setXRot(0);
                     _markers[m]->polygon->setYRot(0);
                     _markers[m]->polygon->setYRot(0);

                    //std::cout << "marker "<< m << " (X,Y,Z)="<< _markers[m]->polygon->getXTrans() << ","<< _markers[m]->polygon->getYTrans()<< ","<< _markers[m]->polygon->getZTrans() << std::endl;



			    }
			    //buffers
			    SwapBuffers(this->window3D->getHDC());	//Swap Buffers

			}

}


 void IRMA::CreateScenario(GL::Object* obj){
        obj->insertNewVertex(2.5f, 2.5f, 5.0f);
		obj->insertNewVertex(2.5f, -2.5f, 5.0f);
		obj->insertNewVertex(2.5f, -2.5f, 5.0f);
		obj->insertNewVertex(-2.5f, -2.5f, 5.0f);
		obj->insertNewVertex(-2.5f, -2.5f, 5.0f);
		obj->insertNewVertex(-2.5f, 2.5f, 5.0f);
		obj->insertNewVertex(-2.5f, 2.5f, 5.0f);
		obj->insertNewVertex(2.5f, 2.5f, 5.0f);
		obj->insertNewVertex(2.5f, 2.5f, -5.0f);
		obj->insertNewVertex(2.5f, -2.5f, -5.0f);
		obj->insertNewVertex(2.5f, -2.5f, -5.0f);
		obj->insertNewVertex(-2.5f, -2.5f, -5.0f);
		obj->insertNewVertex(-2.5f, -2.5f, -5.0f);
		obj->insertNewVertex(-2.5f, 2.5f, -5.0f);
		obj->insertNewVertex(-2.5f, 2.5f, -5.0f);
		obj->insertNewVertex(2.5f, 2.5f, -5.0f);
		obj->insertNewVertex(2.5f, 2.5f, 5.0f);
		obj->insertNewVertex(2.5f, 2.5f, -5.0f);
		obj->insertNewVertex(2.5f, -2.5f, 5.0f);
		obj->insertNewVertex(2.5f, -2.5f, -5.0f);
		obj->insertNewVertex(-2.5f, -2.5f, 5.0f);
		obj->insertNewVertex(-2.5f, -2.5f, -5.0f);
		obj->insertNewVertex(-2.5f, 2.5f, 5.0f);
		obj->insertNewVertex(-2.5f, 2.5f, -5.0f);

 }

  void IRMA::CreatePolygon(GL::Object* obj,int vertnum ){

      switch (vertnum){

        case 3:
            //triangle
        obj->insertNewVertex(0.0f,1.0f,0.0f);
        obj->insertNewVertex(0.5f,0.0f,0.0f);

        obj->insertNewVertex(-0.5f,0.0f,0.0f);
        obj->insertNewVertex(0.5f,0.0f,0.0f);

        obj->insertNewVertex(-0.5f,0.0f,0.0f);
        obj->insertNewVertex(0.0f,1.0f,0.0f);

            break;
        case 4:
            //squad
        obj->insertNewVertex(-1.0f,-1.0f,0.0f);
        obj->insertNewVertex(1.0f,-1.0f,0.0f);

        obj->insertNewVertex(1.0f,-1.0f,0.0f);
        obj->insertNewVertex(1.0f,1.0f,0.0f);

        obj->insertNewVertex(1.0f,1.0f,0.0f);
        obj->insertNewVertex(-1.0f,1.0f,0.0f);

        obj->insertNewVertex(-1.0f,1.0f,0.0f);
         obj->insertNewVertex(-1.0f,-1.0f,0.0f);

            break;
        case 5:
             //pentagon
        obj->insertNewVertex(0.0f,-1.5f,0.0f);
        obj->insertNewVertex(1.0f,-1.0f,0.0f);


        obj->insertNewVertex(1.0f,-1.0f,0.0f);
        obj->insertNewVertex(1.0f,1.0f,0.0f);

        obj->insertNewVertex(1.0f,1.0f,0.0f);
        obj->insertNewVertex(-1.0f,1.0f,0.0f);

        obj->insertNewVertex(-1.0f,1.0f,0.0f);
         obj->insertNewVertex(-1.0f,-1.0f,0.0f);

        obj->insertNewVertex(-1.0f,-1.0f,0.0f);
        obj->insertNewVertex(0.0f,-1.5f,0.0f);

            break;
        case 6:
            //hexagon
        obj->insertNewVertex(0.0f,-1.5f,0.0f);
        obj->insertNewVertex(1.0f,-1.0f,0.0f);

        obj->insertNewVertex(1.0f,-1.0f,0.0f);
        obj->insertNewVertex(1.0f,1.0f,0.0f);

        obj->insertNewVertex(1.0f,1.0f,0.0f);
        obj->insertNewVertex(0.0f,1.5f,0.0f);

        obj->insertNewVertex(0.0f,1.5f,0.0f);
        obj->insertNewVertex(-1.0f,1.0f,0.0f);

        obj->insertNewVertex(-1.0f,1.0f,0.0f);
         obj->insertNewVertex(-1.0f,-1.0f,0.0f);

        obj->insertNewVertex(-1.0f,-1.0f,0.0f);
        obj->insertNewVertex(0.0f,-1.5f,0.0f);

            break;

      }


  }


