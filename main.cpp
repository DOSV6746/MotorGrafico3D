#include <allegro.h>
#include "geometria3D.h"
#include <fstream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

using namespace std;

const int ventana_x = 1000;
const int ventana_y = 500;
double puntocentralx = ventana_x/2;
double puntocentraly = ventana_y/2;

struct rgb {
    int r,g,b;
};

struct punto2D {
    double x;
    double y;
    vector_origen vision;
    //bool adelante = false;

    void proyeccion(vector_origen pos_cam, vector_origen punto, double gx, double gy);
    void dibujar(BITMAP *bu, int color);
};

void punto2D::proyeccion(vector_origen pos_cam, vector_origen punto, double gx, double gy) {

    vector_origen visiono = {1,0,0},visionaux;

    recta lu;

    lu.punto_de_recta.es_igual_a({0,0,0});

    visiono.rotar_punto_respecto_eje('z',gx,visionaux);
    lu.vector_de_direccion.es_igual_a({-visionaux.y,visionaux.x,0});
    rotar_punto_respecto_recta(visionaux,lu,-gy,vision);

    int pol;

    if((gy > M_PI_2 && gy < 1.5*M_PI) || (gy < -M_PI_2 && gy > -1.5*M_PI)) pol = -1;
    else pol = 1;

    plano fotoplano;

    double zoom = 1000;

    pos_cam.mas({zoom*vision.x,zoom*vision.y,zoom*vision.z},fotoplano.punto_de_plano);
    fotoplano.vector_normal.es_igual_a(vision);

    vector_origen vper = {-vision.y,vision.x,0};

    recta lper;

    lper.punto_de_recta.es_igual_a(fotoplano.punto_de_plano);
    lper.vector_de_direccion.es_igual_a(vper);


    recta l;
    l.punto_de_recta.es_igual_a(pos_cam);
    punto.menos(pos_cam,l.vector_de_direccion);

    vector_origen interseccion;
    interseccion_recta_plano(l,fotoplano,interseccion);

    plano raux;

    raux.punto_de_plano.es_igual_a(interseccion);
    raux.vector_normal.es_igual_a(lper.vector_de_direccion);

    vector_origen interseccion_con_lper;
    interseccion_recta_plano(lper,raux,interseccion_con_lper);

    vector_origen vx,vy;

    interseccion_con_lper.menos(fotoplano.punto_de_plano,vx);
    interseccion.menos(interseccion_con_lper,vy);

    int pvx = 1,pvy = 1;

    plano rpvx;

    rpvx.punto_de_plano.es_igual_a(fotoplano.punto_de_plano);
    rpvx.vector_normal.es_igual_a(lper.vector_de_direccion);

    if(adelante(rpvx,interseccion)) pvx = 1;
    else pvx = -1;

    plano rpvy;

    rpvy.punto_de_plano.es_igual_a(fotoplano.punto_de_plano);

    vector_origen A,B;
    fotoplano.punto_de_plano.menos(pos_cam,A);
    interseccion_con_lper.menos(pos_cam,B);
    producto_cruz(A,B,rpvy.vector_normal);

    if(adelante(rpvy,interseccion)) pvy = 1;
    else pvy = -1;

    this->x = puntocentralx + (pol*pvx*vx.valor_absoluto());
    this->y = puntocentraly - (pol*pvy*pvx*vy.valor_absoluto());
}

void punto2D::dibujar(BITMAP *bu, int color) {
    putpixel(bu,x,y,color);
}

class instancia {
public:
    vector_origen centro;
    vector<vector_origen> vertice;
    vector<triangulo> poligono;
    string modelo;
    BITMAP *bu;
    vector_origen vision;
    rgb color;
    bool sombra = false;

    void getCentro(vector_origen &c);
    void setCentro(vector_origen c);
    int getNumeroDeVertices();
    void setVertice(int n_v,vector_origen vec);
    //void setMatrizDeUnion(int i, int j, bool nodo);
    void setTriangulo(int n_t, int v1, int v2, int v3);
    void getVertice(int n_v, vector_origen &vuelto);
    int getNumeroDeTriangulos();
    //bool getMatrizDeUnion(int i, int j);
    void rotar_respecto_eje(char eje, double grados);
    void rotar_respecto_recta(recta recta_de_direccion, double grados);
    void rotar_vertices_respecto_recta(recta recta_de_direccion, double grados);
    void cargar_modelo_obj(string nombre_de_archivo);
    instancia(vector_origen _centro, string madelo, rgb _color, BITMAP *_bu);
    void dibujar(vector_origen pers, double gx, double gy, int r, int g, int b, bool sombra);
    void getVision(vector_origen &r);
    ~instancia();
};

instancia::instancia(vector_origen _centro,string _modelo, rgb _color, BITMAP *_bu) {
    centro = _centro;
    bu = _bu;
    modelo = _modelo;
    cargar_modelo_obj(modelo);
    color = _color;
}

void instancia::getCentro(vector_origen &c) {
    c.es_igual_a(centro);
}

void instancia::setCentro(vector_origen c) {
    centro.es_igual_a(c);
}

int instancia::getNumeroDeVertices() {
    return vertice.size();
}

void instancia::setVertice(int n_v,vector_origen vec) {
    vertice[n_v] = vec;
}

void instancia::setTriangulo(int n_t, int v1, int v2, int v3) {
    poligono[n_t].vertice[0].es_igual_a(vertice[v1]);
    poligono[n_t].vertice[1].es_igual_a(vertice[v2]);
    poligono[n_t].vertice[2].es_igual_a(vertice[v3]);
}

void instancia::getVertice(int n_v, vector_origen &vuelto) {
    vuelto.es_igual_a(vertice[n_v]);
}

int instancia::getNumeroDeTriangulos() {
    return poligono.size();
}

void instancia::rotar_respecto_eje(char eje, double grados) {
    for(int i = 0; i < poligono.size(); i++) {
        for(int j = 0; j < 3; j++) poligono[i].vertice[j].rotar_punto_respecto_eje(eje,grados,poligono[i].vertice[j]);
    }
}

void instancia::rotar_respecto_recta(recta recta_de_direccion, double grados) {
    rotar_punto_respecto_recta(centro,recta_de_direccion,grados,centro);
}

void instancia::rotar_vertices_respecto_recta(recta recta_de_direccion, double grados) {
    for(int i = 0; i < vertice.size(); i++) {
        for(int j = 0; j < 3; j++) rotar_punto_respecto_recta(poligono[i].vertice[j],recta_de_direccion,grados,poligono[i].vertice[j]);
    }
}

void instancia::cargar_modelo_obj(string nombre_de_archivo) {

    vertice.resize(0);
    poligono.resize(0);

    string linea,strnumero = "";

    ifstream mano;

    mano.open(nombre_de_archivo,ios::in);

    if(mano.fail()) {
        printf("No se ha podido abrir o en encontrar el archivo");
        exit(1);
    }

    int cv = 0,cc = 0;
    while(!mano.eof()) {
        getline(mano,linea);

        int j = 0;

        if(linea[0] == 'v' && linea[1] == ' ') {
            for(int i = 0; i < linea.size(); i++) {
                if(linea[i] != 'v' && linea[i] != ' ') {
                    strnumero += linea[i];
                }
                if(linea[i] == ' ' || i == linea.size() - 1) {
                    double v[3];
                    if(strnumero.size() > 0 && j < 2) {
                        v[j] = stof(strnumero);
                        j++;
                    } else if(strnumero.size() > 0) {
                        v[j] = stof(strnumero);
                        vertice.push_back({v[0],v[1],v[2]});
                        j = 0;
                    }

                    strnumero = "";
                }
            }
            cv++;
        }

        if(linea[0] == 'f' && linea[1] == ' ') {
            for(int i = 0; i < linea.size(); i++) {
                if(linea[i] != 'f' && linea[i] != ' ' && linea[i] != '/') {
                    strnumero += linea[i];
                }
                if(linea[i] == ' ' || i == linea.size() - 1 || linea[i] == '/') {
                    //if(strnumero.size() > 0) printf("%i ",stoi(strnumero));
                    int vt[3];
                    if(strnumero.size() > 0 && j < 2) {
                        vt[j] = stoi(strnumero)-1;
                        j++;
                    } else if(strnumero.size() > 0) {
                        vt[j] = stoi(strnumero)-1;
                        poligono.push_back({vertice.at(vt[0]),vertice.at(vt[1]),vertice.at(vt[2])});
                        j = 0;
                        cc++;
                    }

                    strnumero = "";
                }
            }
        }
    }

    mano.close();
}

void instancia::dibujar(vector_origen pers, double gx, double gy, int r, int g, int b, bool sombra) {

    for(int i = 0; i < poligono.size(); i++) {
        for(int j = 1; j < poligono.size(); j++) {
            vector_origen promedio1,promedio2;

            promedio1.es_igual_a(poligono[j-1].vertice[0]);
            promedio1.mas(poligono[j-1].vertice[1],promedio1);
            promedio1.mas(poligono[j-1].vertice[2],promedio1);
            promedio1 = {promedio1.x/3.0,promedio1.y/3.0,promedio1.z/3.0};

            promedio2.es_igual_a(poligono[j].vertice[0]);
            promedio2.mas(poligono[j].vertice[1],promedio1);
            promedio2.mas(poligono[j].vertice[2],promedio1);
            promedio2 = {promedio2.x/3.0,promedio2.y/3.0,promedio2.z/3.0};

            promedio1.mas(pers,promedio1);
            promedio2.mas(pers,promedio2);

            if(promedio1.valor_absoluto() < promedio2.valor_absoluto()) {
                triangulo aux;

                for(int r = 0; r < 3; r++) aux.vertice[r].es_igual_a(poligono[j-1].vertice[r]);
                for(int r = 0; r < 3; r++) poligono[j-1].vertice[r].es_igual_a(poligono[j].vertice[r]);
                for(int r = 0; r < 3; r++) poligono[j].vertice[r].es_igual_a(aux.vertice[r]);

            }
        }
    }

    for(int i = 0; i < poligono.size();i++) {

        punto2D vp[3];
        vector_origen nr;

        poligono[i].getNormal(nr);

        vector_origen aux;

        bool candado = true;
        int cuantos_fuera = 0;
        int cuales[2];
        int contcua = 0;
        int cualesd[2];
        int contcuad = 0;

        punto2D punton[2];

        for(int e = 0; e < 3; e++) {
            plano ra;

            centro.mas(poligono[i].vertice[e],aux);
            vp[e].proyeccion(pers,aux,gx,gy);

            vision.es_igual_a(vp[0].vision);

            ra.punto_de_plano.es_igual_a(pers);
            ra.vector_normal.es_igual_a(vp[e].vision);

            if(adelante(ra,aux) == false) candado = false;

            if((vp[e].x < 0 || vp[e].y < 0) || (vp[e].x > ventana_x || vp[e].y > ventana_y)) {
                cuantos_fuera++;
                cuales[contcua] = e;
                contcua++;
            } else {
                cualesd[contcuad] = e;
                contcuad++;
            }
        }

        if(cuantos_fuera == 1) {

            vector_origen interseccion1,interseccion2;

            int otro1,otro2;

            otro1 = cuales[0]+1;
            otro2 = cuales[0]+2;

            if(otro1 > 2) otro1 -= 3;
            if(otro2 > 2) otro2 -= 3;

            if(vp[cuales[0]].x < 0) {
                recta l1,l2;

                l1.punto_de_recta = {vp[cuales[0]].x,vp[cuales[0]].y,0};
                l1.vector_de_direccion = {vp[cuales[0]].x - vp[otro1].x,vp[cuales[0]].y - vp[otro1].y,0};

                l2.punto_de_recta = {0,0,0};
                l2.vector_de_direccion = {0,1,0};

                interseccion_recta_recta(l1,l2,interseccion1);

                l1.punto_de_recta = {vp[cuales[0]].x,vp[cuales[0]].y,0};
                l1.vector_de_direccion = {vp[cuales[0]].x - vp[otro2].x,vp[cuales[0]].y - vp[otro2].y,0};

                l2.punto_de_recta = {0,0,0};
                l2.vector_de_direccion = {0,1,0};

                interseccion_recta_recta(l1,l2,interseccion2);
            }
            if(vp[cuales[0]].x > ventana_x) {
                recta l1,l2;

                l1.punto_de_recta = {vp[cuales[0]].x,vp[cuales[0]].y,0};
                l1.vector_de_direccion = {vp[cuales[0]].x - vp[otro1].x,vp[cuales[0]].y - vp[otro1].y,0};

                l2.punto_de_recta = {ventana_x,0,0};
                l2.vector_de_direccion = {0,1,0};

                interseccion_recta_recta(l1,l2,interseccion1);

                l1.punto_de_recta = {vp[cuales[0]].x,vp[cuales[0]].y,0};
                l1.vector_de_direccion = {vp[cuales[0]].x - vp[otro2].x,vp[cuales[0]].y - vp[otro2].y,0};

                l2.punto_de_recta = {ventana_x,0,0};
                l2.vector_de_direccion = {0,1,0};

                interseccion_recta_recta(l1,l2,interseccion2);
            }
            if(vp[cuales[0]].y < 0) {
                recta l1,l2;

                l1.punto_de_recta = {vp[cuales[0]].x,vp[cuales[0]].y,0};
                l1.vector_de_direccion = {vp[cuales[0]].x - vp[otro1].x,vp[cuales[0]].y - vp[otro1].y,0};

                l2.punto_de_recta = {0,0,0};
                l2.vector_de_direccion = {1,0,0};

                interseccion_recta_recta(l1,l2,interseccion1);

                l1.punto_de_recta = {vp[cuales[0]].x,vp[cuales[0]].y,0};
                l1.vector_de_direccion = {vp[cuales[0]].x - vp[otro2].x,vp[cuales[0]].y - vp[otro2].y,0};

                l2.punto_de_recta = {0,0,0};
                l2.vector_de_direccion = {1,0,0};

                interseccion_recta_recta(l1,l2,interseccion2);
            }
            if(vp[cuales[0]].y > ventana_y) {
                recta l1,l2;

                l1.punto_de_recta = {vp[cuales[0]].x,vp[cuales[0]].y,0};
                l1.vector_de_direccion = {vp[cuales[0]].x - vp[otro1].x,vp[cuales[0]].y - vp[otro1].y,0};

                l2.punto_de_recta = {0,ventana_y,0};
                l2.vector_de_direccion = {1,0,0};

                interseccion_recta_recta(l1,l2,interseccion1);

                l1.punto_de_recta = {vp[cuales[0]].x,vp[cuales[0]].y,0};
                l1.vector_de_direccion = {vp[cuales[0]].x - vp[otro2].x,vp[cuales[0]].y - vp[otro2].y,0};

                l2.punto_de_recta = {0,ventana_y,0};
                l2.vector_de_direccion = {1,0,0};

                interseccion_recta_recta(l1,l2,interseccion2);
            }


            punton[0].x = interseccion1.x;
            punton[0].y = interseccion1.y;
            punton[1].x = interseccion2.x;
            punton[1].y = interseccion2.y;

        }

        vector_origen fuente_de_luz = {0,0,1000};

        plano cara;

        vector_origen suma;

        centro.mas(poligono[i].vertice[0],suma);

        cara.punto_de_plano.es_igual_a(suma);
        cara.vector_normal.es_igual_a(nr);

        vector_origen vecppp;

        fuente_de_luz.menos(suma,vecppp);

        double prod = producto_punto(nr,vecppp);

        double aji;
        if(prod > (nr.valor_absoluto()*vecppp.valor_absoluto())/7) aji = prod/(nr.valor_absoluto()*vecppp.valor_absoluto());
        else aji = ((nr.valor_absoluto()*vecppp.valor_absoluto())/7)/(nr.valor_absoluto()*vecppp.valor_absoluto());

        int col;

        if(sombra == true) col = makecol(r*aji,g*aji,b*aji);
        else col = makecol(r,g,b);


        if(adelante(cara,pers) && candado && cuantos_fuera == 0) triangle(bu,vp[0].x,vp[0].y,vp[1].x,vp[1].y,vp[2].x,vp[2].y,col);
        if(adelante(cara,pers) && candado && cuantos_fuera == 1) {
            int auxade1,auxade2;

            auxade1 = cuales[0] + 1;
            auxade2 = cuales[0] + 2;

            if(auxade1 > 2) auxade1 -= 3;
            if(auxade2 > 2) auxade2 -= 3;

            triangle(bu,punton[0].x,punton[0].y,vp[auxade1].x,vp[auxade1].y,vp[auxade2].x,vp[auxade2].y,col);
            triangle(bu,punton[0].x,punton[0].y,punton[1].x,punton[1].y,vp[auxade2].x,vp[auxade2].y,col);

        }

        if(adelante(cara,pers) && candado && cuantos_fuera >= 2) {



            segmento borde[4];

            recta l[4];

            borde[0].i = {0,0,0};
            borde[0].f = {ventana_x,0,0};

            l[0].punto_de_recta.es_igual_a(borde[0].i);
            borde[0].f.menos(borde[0].i,l[0].vector_de_direccion);

            borde[1].i = {ventana_x,0,0};
            borde[1].f = {ventana_x,ventana_y,0};

            l[1].punto_de_recta.es_igual_a(borde[1].i);
            borde[1].f.menos(borde[1].i,l[1].vector_de_direccion);

            borde[2].i = {ventana_x,ventana_y,0};
            borde[2].f = {0,ventana_y,0};

            l[2].punto_de_recta.es_igual_a(borde[2].i);
            borde[2].f.menos(borde[2].i,l[2].vector_de_direccion);

            borde[3].i = {0,ventana_y,0};
            borde[3].f = {0,0,0};

            l[3].punto_de_recta.es_igual_a(borde[3].i);
            borde[3].f.menos(borde[3].i,l[3].vector_de_direccion);

            segmento arista[3];
            recta la[3];

            arista[0].i = {vp[0].x,vp[0].y,0};
            arista[0].f = {vp[1].x,vp[1].y,0};

            la[0].punto_de_recta.es_igual_a(arista[0].i);
            arista[0].f.menos(arista[0].i,la[0].vector_de_direccion);

            arista[1].i = {vp[1].x,vp[1].y,0};
            arista[1].f = {vp[2].x,vp[2].y,0};

            la[1].punto_de_recta.es_igual_a(arista[1].i);
            arista[1].f.menos(arista[1].i,la[1].vector_de_direccion);

            arista[2].i = {vp[2].x,vp[2].y,0};
            arista[2].f = {vp[0].x,vp[0].y,0};

            la[2].punto_de_recta.es_igual_a(arista[2].i);
            arista[2].f.menos(arista[2].i,la[2].vector_de_direccion);

            int poarr = 0;
            vector_origen interseccion[120];

            for(int k = 0; k < 3; k++) {
                for(int o = 0; o < 4; o++) {
                    interseccion_recta_recta(la[k],l[o],interseccion[poarr]);

                    if(arista[k].pertenece_al_segmento(interseccion[poarr]) && borde[o].pertenece_al_segmento(interseccion[poarr])) poarr++;
                }
            }

            if(poarr > 0) triangle(bu,vp[0].x,vp[0].y,vp[1].x,vp[1].y,vp[2].x,vp[2].y,col);

        }

    }
}

void instancia::getVision(vector_origen &r) {
    r.es_igual_a(vision);
}

instancia::~instancia() {}

int main() {

    bool game_over = false;

    vector_origen ao = {1,0,0};
    vector_origen aaux;
    vector_origen a;

    vector_origen p = {-1000,0,0};

    double v  = 20;

    double grados_camara_x = 0.1,grados_camara_y = 0.1, grados_camara_z = 0;

    bool disparar = false;

    allegro_init();
    install_timer();
    install_keyboard();
    install_mouse();
    set_color_depth(32);
    set_gfx_mode(GFX_AUTODETECT_WINDOWED, ventana_x, ventana_y, 0, 0);
    BITMAP *buffer = create_bitmap(ventana_x,ventana_y);

    if(install_sound(DIGI_AUTODETECT,MIDI_AUTODETECT, NULL) != 0) {
        allegro_message("Error en sonido\n%s\n", allegro_error);
        return 1;
    }
    set_volume(70,70);

    SAMPLE *sonido_de_fondo = load_wav("disparo.wav");

    show_mouse(NULL);
    set_mouse_speed(2,2);

    srand(time(NULL));

    int negativopositivo;

    if(rand()%2 == 0) negativopositivo = 1;
    else negativopositivo = -1;

    vector<instancia> nave;

    for(int i = 0; i < 5; i++) nave.push_back(instancia({1000+(negativopositivo*rand()%5001),negativopositivo*rand()%5001,negativopositivo*rand()%5001},"nave.txt",{255,0,0},buffer));

    double vnave = 5;

    instancia planeta = instancia({-250,-250,0},"planeta.obj",{255,255,255},buffer);

    const int maxvidaplaneta = 10;
    int vidaplaneta = 10;

    instancia estrella = instancia({1000,0,0},"planeta.obj",{255,255,255},buffer);

    vector<triangulo> hipermesh;
    vector<rgb> hipercolor;
    vector<bool> hipersombra;

    while(!key[KEY_ESC] && !game_over) {

        hipermesh.resize(0);
        hipercolor.resize(0);
        hipersombra.resize(0);

        rectfill(buffer,0,0,ventana_x, ventana_y, 0x000000);

        int pk = 1;

        if((grados_camara_y > M_PI_2 && grados_camara_y < 1.5*M_PI) || (grados_camara_y < -M_PI_2 && grados_camara_y > -1.5*M_PI)) pk = -1;
        else pk = 1;

        grados_camara_x += (mouse_x - puntocentralx)*pk*0.01;

        grados_camara_y -= (mouse_y - puntocentraly)*0.01;


        position_mouse(puntocentralx,puntocentraly);

        if(key[KEY_TAB]) {
            p.z -= v;
        }
        if(key[KEY_SPACE]) {
            p.z += v;
        }

       double perpendicularx = sin(grados_camara_x);
        double perpendiculary = -1*cos(grados_camara_x);

        if(key[KEY_D]) {
            p.x -= v*perpendicularx;
            p.y -= v*perpendiculary;
        }
        if(key[KEY_A]) {
            p.x += v*perpendicularx;
            p.y += v*perpendiculary;
        }

        if(grados_camara_x >= 2*M_PI || grados_camara_x < 0) grados_camara_x = grados_camara_x - (int(grados_camara_x/(2*M_PI))*2*M_PI);
        if(grados_camara_y >= 2*M_PI || grados_camara_y < 0) grados_camara_y = grados_camara_y - (int(grados_camara_y/(2*M_PI))*2*M_PI);

        for(int j = 0; j < nave.size(); j++) {

            vector_origen navedir;

            planeta.centro.menos(nave[j].centro,navedir);

            double k = 1.0/navedir.valor_absoluto();

            navedir = {navedir.x*k,navedir.y*k,navedir.z*k};

            nave[j].centro.mas({vnave*navedir.x,vnave*navedir.y,vnave*navedir.z},nave[j].centro);

            if(abs(atan2(nave[j].poligono[0].vertice[0].y,nave[j].poligono[0].vertice[0].x) - atan2(navedir.y,navedir.x)) > 0.01) nave[j].rotar_respecto_eje('z',-atan2(nave[j].poligono[0].vertice[0].y,nave[j].poligono[0].vertice[0].x) + atan2(navedir.y,navedir.x));

            recta lao;

            lao.punto_de_recta = {0,0,0};
            lao.vector_de_direccion = {-nave[j].poligono[0].vertice[0].y,nave[j].poligono[0].vertice[0].x,0};

            if(-asin(navedir.z/sqrt(pow(navedir.x,2)+pow(navedir.y,2))) + asin(nave[j].poligono[0].vertice[0].z/sqrt(pow(nave[j].poligono[0].vertice[0].x,2) + pow(nave[j].poligono[0].vertice[0].y,2))) > 0.01) nave[j].rotar_vertices_respecto_recta(lao,-asin(navedir.z/sqrt(pow(navedir.x,2)+pow(navedir.y,2))) + asin(nave[j].poligono[0].vertice[0].z/sqrt(pow(nave[j].poligono[0].vertice[0].x,2) + pow(nave[j].poligono[0].vertice[0].y,2))));

            for(int i = 0; i < nave[j].poligono.size(); i++) {

                triangulo trisum;

                for(int g = 0; g < 3; g++) nave[j].centro.mas(nave[j].poligono[i].vertice[g],trisum.vertice[g]);


                hipermesh.push_back(trisum);
                hipercolor.push_back(nave[j].color);
                hipersombra.push_back(true);
            }
        }

        ao.rotar_punto_respecto_eje('z',grados_camara_x,aaux);

        recta lperpen;

        lperpen.punto_de_recta.es_igual_a({0,0,0});
        lperpen.vector_de_direccion = {-aaux.y,aaux.x,0};

        rotar_punto_respecto_recta(aaux,lperpen,-grados_camara_y,a);


        if(key[KEY_W]) {
            p.x += a.x*v;
            p.y += a.y*v;
            p.z += a.z*v;
        }
        if(key[KEY_S]) {
            p.x -= a.x*v;
            p.y -= a.y*v;
            p.z -= a.z*v;
        }

        for(int i = 0; i < planeta.poligono.size(); i++) {
            triangulo trisum;

            for(int g = 0; g < 3; g++) estrella.centro.mas(estrella.poligono[i].vertice[g],trisum.vertice[g]);

            hipermesh.push_back(trisum);
            hipercolor.push_back(estrella.color);
            hipersombra.push_back(false);
        }

        estrella.rotar_respecto_eje('z',0.1);

        for(int i = 0; i < planeta.poligono.size(); i++) {
            triangulo trisum;

            for(int g = 0; g < 3; g++) planeta.centro.mas(planeta.poligono[i].vertice[g],trisum.vertice[g]);

            hipermesh.push_back(trisum);
            hipercolor.push_back(planeta.color);
            hipersombra.push_back(true);
        }

        /*planeta.rotar_respecto_eje('x',0.01);
        planeta.rotar_respecto_eje('y',0.01);
        planeta.rotar_respecto_eje('z',0.01);*/

        recta eje_estrella;

        vector_origen pdif;

        //estrella.centro.menos(planeta.centro,pdif);

        eje_estrella.punto_de_recta.es_igual_a(estrella.centro);

        eje_estrella.vector_de_direccion = {0,0,1};

        planeta.rotar_respecto_recta(eje_estrella,0.001);
        planeta.rotar_respecto_eje('z',-0.01);

        vector_origen voper;

        for(int i = 0; i < nave.size(); i++) {
            nave[i].centro.menos(planeta.centro,voper);
            if(voper.valor_absoluto() < 50) {
                    nave.erase(nave.begin() + i);
                    vidaplaneta--;
            }
        }

        planeta.color = {255,int((double(vidaplaneta-1)/double(maxvidaplaneta-1))*255.0),int((double(vidaplaneta-1)/double(maxvidaplaneta-1))*255.0)};

        if(nave.size() == 0) {
            srand(time(NULL));

            for(int i = rand()%11; i > 0; i--) nave.push_back(instancia({1000+(negativopositivo*rand()%5001),negativopositivo*rand()%5001,negativopositivo*rand()%5001},"nave.txt",{255,0,0},buffer));
        }

        for(int i = 0; i < nave.size(); i++) {
            nave[i].centro.menos(p,voper);
            if(voper.valor_absoluto() < 30) {
                    nave.erase(nave.begin() + i);
            }
        }

        estrella.centro.menos(p,voper);
        if(voper.valor_absoluto() < 90) game_over = true;
        planeta.centro.menos(p,voper);
        if(voper.valor_absoluto() < 90) game_over = true;

        //hiper dibujar

        for(int i = 0; i < hipermesh.size(); i++) {
            for(int j = 1; j < hipermesh.size(); j++) {
                vector_origen promedio1,promedio2;

                promedio1.es_igual_a(hipermesh[j-1].vertice[0]);
                promedio1.mas(hipermesh[j-1].vertice[1],promedio1);
                promedio1.mas(hipermesh[j-1].vertice[2],promedio1);
                promedio1 = {promedio1.x/3.0,promedio1.y/3.0,promedio1.z/3.0};

                promedio2.es_igual_a(hipermesh[j].vertice[0]);
                promedio2.mas(hipermesh[j].vertice[1],promedio2);
                promedio2.mas(hipermesh[j].vertice[2],promedio2);
                promedio2 = {promedio2.x/3.0,promedio2.y/3.0,promedio2.z/3.0};

                promedio1.menos(p,promedio1);
                promedio2.menos(p,promedio2);

                if(promedio1.valor_absoluto() < promedio2.valor_absoluto()) {
                    triangulo aux;

                    for(int r = 0; r < 3; r++) aux.vertice[r].es_igual_a(hipermesh[j-1].vertice[r]);
                    for(int r = 0; r < 3; r++) hipermesh[j-1].vertice[r].es_igual_a(hipermesh[j].vertice[r]);
                    for(int r = 0; r < 3; r++) hipermesh[j].vertice[r].es_igual_a(aux.vertice[r]);

                    rgb caux;

                    caux = hipercolor[j-1];
                    hipercolor[j-1] = hipercolor[j];
                    hipercolor[j] = caux;

                    bool boaux;

                    boaux = hipersombra[j-1];
                    hipersombra[j-1] = hipersombra[j];
                    hipersombra[j] = boaux;

                }
            }
        }

        for(int i = 0; i < hipermesh.size(); i++) {
            punto2D proyeccion_de_vertice[3];

            bool candado = false;
            int cuantos_fuera = 0;
            int cuales[2];
            int contcua = 0;
            int cualesd[2];
            int contcuad = 0;

            for(int k = 0; k < 3; k++) {
                plano ra;

                ra.punto_de_plano.es_igual_a(p);
                ra.vector_normal.es_igual_a(a);

                if(adelante(ra,hipermesh[i].vertice[0]) && adelante(ra,hipermesh[i].vertice[1]) && adelante(ra,hipermesh[i].vertice[2])) candado = true;
                else candado = false;


                proyeccion_de_vertice[k].proyeccion(p,hipermesh[i].vertice[k],grados_camara_x,grados_camara_y);

                if((proyeccion_de_vertice[k].x < 0 || proyeccion_de_vertice[k].y < 0) || (proyeccion_de_vertice[k].x > ventana_x || proyeccion_de_vertice[k].y > ventana_y)) {
                    cuantos_fuera++;
                    cuales[contcua] = k;
                    contcua++;
                } else {
                    cualesd[contcuad] = k;
                    contcuad++;
                }
            }

            vector_origen nr;

            hipermesh[i].getNormal(nr);

            vector_origen fuente_de_luz = estrella.centro;

            plano cara;

            vector_origen suma;

            suma.es_igual_a(hipermesh[i].vertice[0]);

            cara.punto_de_plano.es_igual_a(suma);
            cara.vector_normal.es_igual_a(nr);

            vector_origen vecppp;

            fuente_de_luz.menos(suma,vecppp);

            double prod = producto_punto(nr,vecppp);

            double aji;
            if(prod > (nr.valor_absoluto()*vecppp.valor_absoluto())/7) aji = prod/(nr.valor_absoluto()*vecppp.valor_absoluto());
            else aji = ((nr.valor_absoluto()*vecppp.valor_absoluto())/7)/(nr.valor_absoluto()*vecppp.valor_absoluto());

            int col;

            if(hipersombra[i] == true) col = makecol(hipercolor[i].r*aji,hipercolor[i].g*aji,hipercolor[i].b*aji);
            else col = makecol(hipercolor[i].r,hipercolor[i].g,hipercolor[i].b);


            if(candado == true){

                segmento borde[4];
                recta lb[4];
                vector_origen vdi;

                borde[0].i = {0,0,0};
                borde[0].f = {ventana_x,0,0};

                borde[0].f.menos(borde[0].f,vdi);

                lb[0].punto_de_recta.es_igual_a(borde[0].i);
                lb[0].vector_de_direccion.es_igual_a(vdi);

                borde[1].i = {ventana_x,0,0};
                borde[1].f = {ventana_x,ventana_y,0};

                borde[1].f.menos(borde[1].f,vdi);

                lb[1].punto_de_recta.es_igual_a(borde[1].i);
                lb[1].vector_de_direccion.es_igual_a(vdi);

                borde[2].i = {ventana_x,ventana_y,0};
                borde[2].f = {0,ventana_y,0};

                borde[2].f.menos(borde[2].f,vdi);

                lb[2].punto_de_recta.es_igual_a(borde[2].i);
                lb[2].vector_de_direccion.es_igual_a(vdi);

                borde[3].i = {0,ventana_y,0};
                borde[3].f = {0,0,0};

                borde[3].f.menos(borde[3].f,vdi);

                lb[3].punto_de_recta.es_igual_a(borde[3].i);
                lb[3].vector_de_direccion.es_igual_a(vdi);

                if(cuantos_fuera < 3 && adelante(cara,p)) triangle(buffer,proyeccion_de_vertice[0].x,proyeccion_de_vertice[0].y,proyeccion_de_vertice[1].x,proyeccion_de_vertice[1].y,proyeccion_de_vertice[2].x,proyeccion_de_vertice[2].y,col);
            }
        }

        //fin de hiper dibujar

        putpixel(buffer,puntocentralx,puntocentraly,0xff0000);


        if(vidaplaneta <= 0) game_over = true;

        blit(buffer, screen, 0, 0, 0, 0, ventana_x,ventana_y);
        //rest(10);
    }

    if(game_over) {
        printf("GAME OVER\n");
        printf("Hecho por DOSV6746\n");
    }

    system("PAUSE");

    return 0;

}
END_OF_MAIN();
