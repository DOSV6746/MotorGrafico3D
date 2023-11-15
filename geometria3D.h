#include <math.h>
#include <stdio.h>

signed short int signo(double n) {
    if(n > 0) return 1;
    else if(n < 0) return -1;
    else return 0;
}

struct vector_origen {
    double x,y,z;
    double valor_absoluto();
    void es_igual_a(vector_origen este);
    void mas(vector_origen este, vector_origen &resultado);
    void menos(vector_origen este, vector_origen &resultado);
    void rotar_punto_respecto_eje(char eje, double grados,vector_origen &resultado);
};

double vector_origen::valor_absoluto() {
    return sqrt(pow(x,2) + pow(y,2) + pow(z,2));
}

void vector_origen::es_igual_a(vector_origen este) {
	this->x = este.x;
	this->y = este.y;
	this->z = este.z;
}

void vector_origen::mas(vector_origen este, vector_origen &resultado) {
	resultado.x = this->x + este.x;
	resultado.y = this->y + este.y;
	resultado.z = this->z + este.z;
}

void vector_origen::menos(vector_origen este, vector_origen &resultado) {
	resultado.x = this->x - este.x;
	resultado.y = this->y - este.y;
	resultado.z = this->z - este.z;
}

void vector_origen::rotar_punto_respecto_eje(char eje, double grados, vector_origen &resultado) {
	double r;
	
	switch(eje) {
		case 'x':
			r = sqrt(pow(this->y,2)+pow(this->z,2));
			
			resultado.es_igual_a({this->x, r*cos(grados + atan2(this->z,this->y)), r*sin(grados + atan2(this->z,this->y))});
		break;
		case 'y':
			r = sqrt(pow(this->x,2)+pow(this->z,2));
			
			resultado.es_igual_a({r*cos(grados + atan2(this->z,this->x)), this->y, r*sin(grados + atan2(this->z,this->x))});
		break;
		case 'z':
			r = sqrt(pow(this->x,2)+pow(this->y,2));
			
			resultado.es_igual_a({r*cos(grados + atan2(this->y,this->x)), r*sin(grados + atan2(this->y,this->x)), this->z});
		break;
		default:
			resultado.es_igual_a({this->x,this->y,this->z});
	}
}

double producto_punto(vector_origen vec1,vector_origen vec2) {
    return (vec1.x * vec2.x) + (vec1.y * vec2.y) + (vec1.z * vec2.z);
}

void producto_cruz(vector_origen vec1,vector_origen vec2, vector_origen &resultante) {
    resultante.x = (vec1.y*vec2.z) - (vec2.y*vec1.z);
    resultante.y = -((vec1.x*vec2.z) - (vec2.x*vec1.z));
    resultante.z = (vec1.x*vec2.y) - (vec2.x*vec1.y);
}

struct recta {
    vector_origen punto_de_recta;
    vector_origen vector_de_direccion;
};

struct segmento {
    vector_origen i;
    vector_origen f;

    bool pertenece_al_segmento(vector_origen punto);
};

bool segmento::pertenece_al_segmento(vector_origen punto) {
	
    bool aux,aux2;
    double lam = (punto.x - this->i.x)/(this->f.x - this->i.x);    
    if(lam == 0) lam = (punto.y - this->i.y)/(this->f.y - this->i.y);
    if(lam == 0) lam = (punto.z - this->i.z)/(this->f.z - this->i.z);

    if(punto.y == this->i.y + ((this->f.y - this->i.y)*lam) && punto.z == this->i.z + ((this->f.z - this->i.z)*lam)) aux = true;
    else aux = false;

    vector_origen menor, mayor;

    if(f.x > i.x) {
        menor.x = i.x;
        mayor.x = f.x;
    } else {
        menor.x = f.x;
        mayor.x = i.x;
    }
    if(f.y > i.y) {
        menor.y = i.y;
        mayor.y = f.y;
    } else {
        menor.y = f.y;
        mayor.y = i.y;
    }
    if(f.z > i.z) {
        menor.z = i.z;
        mayor.z = f.z;
    } else {
        menor.z = f.z;
        mayor.z = i.z;
    }

    if(punto.x >= menor.x && punto.x <= mayor.x && punto.y >= menor.y && punto.y <= mayor.y && punto.z >= menor.z && punto.z <= mayor.z) aux2 = true;
    else aux2 = false;

    if(aux && aux2) return true;
    else return false;

}

struct plano {
    vector_origen punto_de_plano;
    vector_origen vector_normal;

    bool pertenece_al_plano(vector_origen punto);
};

void interseccion_recta_plano(recta recta1,plano plano1, vector_origen &punto_de_interseccion) {

    double lam = double(plano1.vector_normal.x*(plano1.punto_de_plano.x - recta1.punto_de_recta.x) + plano1.vector_normal.y*(plano1.punto_de_plano.y - recta1.punto_de_recta.y) + plano1.vector_normal.z*(plano1.punto_de_plano.z - recta1.punto_de_recta.z))/double((plano1.vector_normal.x*recta1.vector_de_direccion.x) + (plano1.vector_normal.y*recta1.vector_de_direccion.y) + (plano1.vector_normal.z*recta1.vector_de_direccion.z));

    punto_de_interseccion.x = recta1.punto_de_recta.x + double(recta1.vector_de_direccion.x*lam);
    punto_de_interseccion.y = recta1.punto_de_recta.y + double(recta1.vector_de_direccion.y*lam);
    punto_de_interseccion.z = recta1.punto_de_recta.z + double(recta1.vector_de_direccion.z*lam);
}

void interseccion_recta_recta(recta recta1,recta recta2, vector_origen &punto_de_interseccion) {
    double divisor = ((recta2.vector_de_direccion.x*recta1.vector_de_direccion.y) - (recta2.vector_de_direccion.y*recta1.vector_de_direccion.x));

    double t;

    if(divisor != 0) t = (recta2.vector_de_direccion.x*(recta2.punto_de_recta.y - recta1.punto_de_recta.y) + recta2.vector_de_direccion.y*(recta1.punto_de_recta.x - recta2.punto_de_recta.x))/divisor;
    else {
        double segundo_divisor = ((recta2.vector_de_direccion.x*recta1.vector_de_direccion.z) - (recta2.vector_de_direccion.z*recta1.vector_de_direccion.x));
        if(segundo_divisor != 0) t = (recta2.vector_de_direccion.x*(recta2.punto_de_recta.z - recta1.punto_de_recta.z) + recta2.vector_de_direccion.z*(recta1.punto_de_recta.x - recta2.punto_de_recta.x))/segundo_divisor;
        else t = (recta2.vector_de_direccion.y*(recta2.punto_de_recta.z - recta1.punto_de_recta.z) + recta2.vector_de_direccion.z*(recta1.punto_de_recta.y - recta2.punto_de_recta.y))/((recta2.vector_de_direccion.y*recta1.vector_de_direccion.z) - (recta2.vector_de_direccion.z*recta1.vector_de_direccion.y));
    }

    punto_de_interseccion.x = recta1.punto_de_recta.x + (recta1.vector_de_direccion.x*t);
    punto_de_interseccion.y = recta1.punto_de_recta.y + (recta1.vector_de_direccion.y*t);
    punto_de_interseccion.z = recta1.punto_de_recta.z + (recta1.vector_de_direccion.z*t);
}

bool intersectan_estas_rectas_en_un_punto(recta recta1, recta recta2) {
    double k;

    if(recta1.vector_de_direccion.x != 0 || recta1.vector_de_direccion.x != 0) k = recta1.vector_de_direccion.x/recta2.vector_de_direccion.x;
    else if(recta1.vector_de_direccion.y != 0 || recta1.vector_de_direccion.y != 0) k = recta1.vector_de_direccion.y/recta2.vector_de_direccion.y;
    else k = recta1.vector_de_direccion.z/recta2.vector_de_direccion.z;

    if(recta1.vector_de_direccion.x == k*recta2.vector_de_direccion.x && recta1.vector_de_direccion.y == k*recta2.vector_de_direccion.y && recta1.vector_de_direccion.z == k*recta2.vector_de_direccion.z) return false;
    else return true;
}

bool adelante(plano plano_a_evaluar, vector_origen punto_a_evaluar) {   
    vector_origen aux;
    
    punto_a_evaluar.menos(plano_a_evaluar.punto_de_plano,aux);
    
    if(producto_punto(plano_a_evaluar.vector_normal,aux) > 0.0) return true;
    else return false;
}

bool plano::pertenece_al_plano(vector_origen punto) {
    vector_origen aux;

    punto.menos(punto_de_plano,aux);
    if(producto_punto(aux,vector_normal) == 0) return true;
    else return false;

}

void rotar_punto_respecto_recta(vector_origen punto, recta recta_direccion, double grados, vector_origen &nuevo_punto) {

     vector_origen interseccion;
     plano plano_de_circunferencia;

     plano_de_circunferencia.punto_de_plano.es_igual_a(punto);
     plano_de_circunferencia.vector_normal.es_igual_a(recta_direccion.vector_de_direccion);

     interseccion_recta_plano(recta_direccion, plano_de_circunferencia, interseccion);

     vector_origen r_v;
     punto.menos(interseccion, r_v);
     double r = r_v.valor_absoluto();

     vector_origen a, b;

     double k = 1/r;
     a.x = k*r_v.x;
     a.y = k*r_v.y;
     a.z = k*r_v.z;

     producto_cruz(plano_de_circunferencia.vector_normal, a, b);
     k = 1/b.valor_absoluto();

     b.x = k*b.x;
     b.y = k*b.y;
     b.z = k*b.z;

     nuevo_punto.x = interseccion.x + r*cos(grados)*a.x + r*sin(grados)*b.x;
     nuevo_punto.y = interseccion.y + r*cos(grados)*a.y + r*sin(grados)*b.y;
     nuevo_punto.z = interseccion.z + r*cos(grados)*a.z + r*sin(grados)*b.z;
}

struct triangulo {
    vector_origen vertice[3];
    
    void getNormal(vector_origen &normal);
    bool pertenece_al_triangulo(vector_origen punto);
};

void triangulo::getNormal(vector_origen &normal) {
	vector_origen vector1, vector2;
	
	vertice[1].menos(vertice[0],vector1);
	vertice[2].menos(vertice[0],vector2);
	
	producto_cruz(vector1,vector2,normal);
}

bool triangulo::pertenece_al_triangulo(vector_origen punto) {
    plano pdf;

    vector_origen r1,r2,re;
    vertice[1].menos(vertice[0],r1);
    vertice[2].menos(vertice[0],r2);
    producto_cruz(r1,r2,re);

    pdf.punto_de_plano.es_igual_a(vertice[0]);
    pdf.vector_normal.es_igual_a(re);

    bool vof, tf[3];
    vector_origen o;
    recta lo, la;
    
    vof = pdf.pertenece_al_plano(punto);

    o.es_igual_a({(vertice[0].x+vertice[1].x+vertice[2].x)/3,(vertice[0].y+vertice[1].y+vertice[2].y)/3,(vertice[0].z+vertice[1].z+vertice[2].z)/3});

    lo.punto_de_recta.es_igual_a(o);
    punto.menos(o,lo.vector_de_direccion);
    
    segmento arista[3];
    segmento c;
    c.i.es_igual_a(o);
    c.f.es_igual_a(punto);

    for(int i = 0; i < 3; i++) {
        if(i < 3) {
            arista[i].i.es_igual_a(vertice[i]);
            arista[i].f.es_igual_a(vertice[i + 1]);
        } else {
            arista[i].i.es_igual_a(vertice[i]);
            arista[i].f.es_igual_a(vertice[0]);
        }
    }

    for(int i = 0; i < 3; i++) {
       la.punto_de_recta.es_igual_a(arista[i].i);
       arista[i].f.menos(arista[i].i,la.vector_de_direccion);

       if(intersectan_estas_rectas_en_un_punto(lo,la)) {
           vector_origen ne;
           interseccion_recta_recta(lo,la,ne);

           if(arista[i].pertenece_al_segmento(ne) && c.pertenece_al_segmento(ne)) {
               tf[i] = true;
           } else tf[i] = false;
       } else tf[i] = false;
    }

    if(tf[0] == true || tf[1] == true || tf[2] == true) vof = true;
    else vof = false;

    if(pdf.pertenece_al_plano(punto) && vof) return true;
    else return false;
}
