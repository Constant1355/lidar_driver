#include <iostream>
#include "displayer.hpp"
void init();
void display();
void reshape(int w, int h);
void processNormalKeys(unsigned char key,int x,int y);
void timer_cb(int value);

velodyne_rawdata::vcloud disp_cloud;
std::mutex disp_mtx;

float camera_pos_x = 0.0f;
float camera_pos_y = 0.0f;
float camera_pos_z = 40.0f;
float camera_rot_x = 0.0f;
float camera_rot_y = 0.0f;
float camera_rot_z = 0.0f;

int GL_show(int argc, char* argv[])
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_SINGLE);
    glutInitWindowPosition(0, 0);
    glutInitWindowSize(300, 300);
    
    glutCreateWindow("OpenGL 3D View");
    
    init();
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(processNormalKeys);
    glutTimerFunc(100,timer_cb,1);
    
    glutMainLoop();
    return 0;
}
void init()
{
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glRotatef(camera_rot_x, 1.0f, 0.0f, 0.0f);
    glRotatef(camera_rot_y, 0.0f, 1.0f, 0.0f);
    glRotatef(camera_rot_z, 0.0f, 0.0f, 1.0f);

    glFrustum(-1,1,-1,1,1,200);
    gluLookAt(camera_pos_x, camera_pos_y, camera_pos_z, 0, 0, 0, 0, 1, 0);
}
void display()
{
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    // glRotatef(1, 0.0f, 0.0f, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);  
    glPointSize(1);
    glBegin(GL_POINTS);
        disp_mtx.lock();
        for(auto p: disp_cloud){
            int ring = std::max(0.0f,std::min(16.0f,p.r));
            float& r = color_table[ring][0];
            float& g = color_table[ring][1];
            float& b = color_table[ring][2];
            glColor3f(r, g, b);
            glVertex3f(p.x,p.y,p.z);
            // glVertex3f(temp, temp, temp);
        }
        disp_mtx.unlock();
    glEnd();
    glFlush();
}

void reshape(int w, int h)
{
   glViewport (0, 0, (GLsizei) w, (GLsizei) h);
   init();
}

void processNormalKeys(unsigned char key,int x,int y)
{
    switch(key){
        case 27:
            exit(0);
            break;
        case 'w':
            camera_pos_y +=1;
            break;
        case 's':
            camera_pos_y -=1;
            break;
        case 'a':
            camera_pos_x -=1;
            break;
        case 'd':
            camera_pos_x +=1;
            break;
    }
    init();
    
        
} 

void timer_cb(int value){
    glutPostRedisplay();
    glutTimerFunc(10,timer_cb,1);
}