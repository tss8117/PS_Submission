#include<iostream>
#include<vector>
#include<algorithm>
#include<ncurses.h>
#include<random>
using namespace std;
random_device rd;
mt19937 gen{rd()};

struct pos{
    int x;
    int y;
    bool operator==(const pos& other) const {
        return x == other.x && y == other.y;
    }
};

class Snake{
private:
    WINDOW* curwin;
    int length;
    vector<pos> position;
    int MaxY, MaxX;
    int direction; 
public:
    Snake(WINDOW* win){
        curwin = win;
        length = 1;
        pos p;
        p.x = 1;
        p.y = 1;
        position.push_back(p);
        getmaxyx(curwin, MaxY, MaxX);
        direction = KEY_RIGHT;
    }
    Snake(){
        curwin = nullptr;
        length = 0;
        MaxY = 0;
        MaxX = 0;
    }
    void eat(pos newPos){
        position.insert(position.begin(), newPos);
        length++;
    }
    void mvup(){
        direction = KEY_UP;
        for(int i=length-1;i>0;i--){
            position[i] = position[i-1];
        }
        position[0].y--;
        if(position[0].y==0) 
            position[0].y=MaxY-2;
    }
    void mvdown(){
        direction = KEY_DOWN;
        for(int i=length-1;i>0;i--){
            position[i] = position[i-1];
        }
        position[0].y++;
        if(position[0].y==MaxY-1) 
            position[0].y=1;
    }
    void mvleft(){
        direction = KEY_LEFT;
        for(int i=length-1;i>0;i--){
            position[i] = position[i-1];
        }
        position[0].x--;
        if(position[0].x==0) 
            position[0].x=MaxX-2;
    }
    void mvright(){
        direction = KEY_RIGHT;
        for(int i=length-1;i>0;i--){
            position[i] = position[i-1];
        }
        position[0].x++;
        if(position[0].x==MaxX-1) 
            position[0].x=1;
    }
    int getlength(){
        return length;
    }
    vector<pos>& getPosition(){
        return position;
    }
    int getDirection(){
        return direction;
    }
};

class Food{
private:
    pos position;
    int maxX, maxY;
public:
    Food(WINDOW* curwin){
        getmaxyx(curwin, maxY, maxX);
        uniform_int_distribution<> distrX(1, maxX-2);
        uniform_int_distribution<> distrY(1, maxY-2);
        position.x = distrX(gen);
        position.y = distrY(gen);
    }
    Food(){
        position.x = 0;
        position.y = 0;
        maxX = 0;
        maxY = 0;
    }
    void display(WINDOW* curwin){
        mvwprintw(curwin, position.y, position.x, "*");
    }
    pos getPosition(){
        return position;
    }

};

class Game{
private:
    WINDOW* curwin;
    Snake s;
    Food f;
public:
    Game(WINDOW* win) : s(win), f(win){
        curwin = win;
        box(curwin, 0, 0);
        mvwprintw(curwin, 1, 1, "Welcome to Snake Game!");
        mvwprintw(curwin, 2, 1, "Press any key to start!");
        refresh();
        wrefresh(curwin);
        getch();
        werase(curwin);
        box(curwin, 0, 0);
        wrefresh(curwin);
        f.display(curwin);
    }
    void loop(WINDOW* curwin){
        while(1){
            box(curwin, 0, 0);
            for(int i=0; i<s.getlength(); i++){
                mvwprintw(curwin, s.getPosition()[i].y, s.getPosition()[i].x, "o");
            }
            if(f.getPosition()==s.getPosition()[0]){
                s.eat(f.getPosition());
                f = Food(curwin);
            }
            f.display(curwin);
            refresh();
            wrefresh(curwin);
            int ch = wgetch(curwin);
            switch(ch){
                case KEY_UP:
                    if(s.getDirection() != KEY_DOWN){
                        s.mvup();
                    }
                    break;
                case KEY_DOWN:
                    if(s.getDirection() != KEY_UP){
                        s.mvdown();
                    }
                    break;
                case KEY_LEFT:
                    if(s.getDirection() != KEY_RIGHT){
                        s.mvleft();
                    }
                    break;
                case KEY_RIGHT:
                    if(s.getDirection() != KEY_LEFT){
                        s.mvright();
                    }
                    break;
                case ERR:   // no key pressed
                    switch(s.getDirection()){
                        case KEY_UP:    
                            s.mvup();    
                            break;
                        case KEY_DOWN:  
                            s.mvdown();  
                            break;
                        case KEY_LEFT:  
                            s.mvleft();  
                            break;
                        case KEY_RIGHT: 
                            s.mvright(); 
                            break;
                    }
                    break;
                default:
                    break;
            }       
            werase(curwin);
            napms(150);  
            if(s.getlength() > 4){
                if(find(s.getPosition().begin()+1, s.getPosition().end(), s.getPosition()[0]) != s.getPosition().end()){
                    box(curwin, 0, 0);
                    mvwprintw(curwin, 1, 1, "Game Over! Your score: %d", s.getlength()-1);
                    mvwprintw(curwin, 2, 1, "Press enter to exit.");
                    refresh();
                    wrefresh(curwin);
                    while(1){
                        int ch = wgetch(curwin);
                        if(ch == 10 || ch == 13 || ch == KEY_ENTER){
                            break;
                        }
                    }
                    break;
                }
            }
        }
    }
};


int main() {

    initscr();
    curs_set(0);
    cbreak();
    noecho();

    int maxY, maxX;
    getmaxyx(stdscr, maxY, maxX);

    WINDOW* win = newwin(2*maxY/3, 2*maxX/3, maxY/6, maxX/6);
    keypad(win, TRUE);
    nodelay(win, TRUE);


    Game game = Game(win);
    game.loop(win);

    delwin(win);
    endwin();
    return 0;
}