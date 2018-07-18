#include <SDL2/SDL.h>
#include <steeriously/libinc.hpp>
#include <stdio.h>

//constants
const int WINDOWX = 800;
const int WINDOWY = 600;
const double WALLPADDING = 20.0;
const int OBSTACLES = 50;
const int MINRADIUS = 10.f;
const int MAXRADIUS = 15.f;
const int MAXITERS = 2000;
const int MINGAP = 4.f;
const int MINBORDER = 50.f;
const int flockNum = 100;
const Uint32 MAXIMUM_FRAME_RATE = 60;
const Uint32 MINIMUM_FRAME_RATE = 30;
const double UPDATE_INTERVAL = (1.0 / MAXIMUM_FRAME_RATE);
const double MAX_CYCLES_PER_FRAME = (MAXIMUM_FRAME_RATE / MINIMUM_FRAME_RATE);

//globals...
SDL_Window* window = NULL;
SDL_Renderer* renderer = NULL;
std::vector<steer::Wall> m_walls;
std::vector<steer::SphereObstacle> m_obstacles;

//prototypes...
bool init();
void createWalls();
void createObstacles();

int main( int argc, char* args[] )
{
    //seed random...
    srand(NULL);

    //set up steering entities...
    //set up walls
    createWalls();

    //set up obstacles
    createObstacles();

    //set up a path
    std::list<steer::Vector2> pathPoints;
    pathPoints.push_back(steer::Vector2(400.0, 100.0));
    pathPoints.push_back(steer::Vector2(200.0, 200.0));
    pathPoints.push_back(steer::Vector2(150.0, 250.0));
    pathPoints.push_back(steer::Vector2(450.0, 300.0));
    pathPoints.push_back(steer::Vector2(400.0, 100.0));
    steer::Path path(5,pathPoints);
    path.loopOn();

    //a flock!!!
    steer::BehaviorParameters fparams;
    fparams.NumAgents = flockNum/2;
    fparams.SeparationWeight = 10.f;
    fparams.AlignmentWeight = 10.f;
    fparams.CohesionWeight = 10.f;
    fparams.SeekWeight = 2.f;
    fparams.WallAvoidanceWeight = 50.f;
    fparams.ObstacleAvoidanceWeight = 50.f;
    fparams.radius = 4.f;
    fparams.MaxForce = 500.f;
    fparams.MaxSpeed = 300.f;
    fparams.DecelerationTweaker = 100.f;
    fparams.MinDetectionBoxLength = 100.f;
    fparams.WallDetectionFeelerLength = 100.f;
    std::vector<steer::SuperComponent> flock;
    std::vector<steer::SuperComponent*> pf;
    std::vector<steer::SuperComponent*> pf2;
    std::vector<SDL_Rect> frects;
    steer::Path tpath(5,pathPoints);
    tpath.loopOn();
    steer::BehaviorParameters pparams;
    pparams.radius = 8.f;
    pparams.FollowPathWeight = 5.f;
    pparams.SeekWeight = 1.f;
    pparams.NumAgents = flockNum/2;
    pparams.SeparationWeight = 1.f;
    pparams.AlignmentWeight = 10.f;
    pparams.CohesionWeight = 10.f;
    pparams.MaxForce = 300.f;
    pparams.MaxSpeed = 150.f;
    pparams.DecelerationTweaker = 100.f;

    for(int i=0; i<flockNum; ++i)
    {
        int tx = rand() % 700;
        int ty = rand() % 500;
        SDL_Rect temp;

        if(i % 2 == 0)//make this segment of the flock follow the path...
        {
            steer::SuperComponent tf(&pparams);
            tf.setTarget(steer::Vector2(400.0, 300.0));
            tf.setPosition(steer::Vector2(tx, ty));
            tf.flockingOn();
            tf.pathFollowingOn();
            tf.obstacleAvoidanceOff();
            tf.wallAvoidanceOff();
            tf.seekOn();
            tf.setPath(&tpath);
            flock.push_back(tf);

            temp.h = pparams.radius;
            temp.w = pparams.radius;
            //temp.setOrigin(pparams.radius/2.f, pparams.radius/2.f);
        }else{//make this segment of the flock follow the mouse...
            steer::SuperComponent tf(&fparams);
            tf.flockingOn();
            tf.setTarget(steer::Vector2(400.0, 300.0));
            tf.setPosition(steer::Vector2(tx, ty));
            flock.push_back(tf);

            temp.h = fparams.radius;
            temp.w = fparams.radius;

            //temp.setOrigin(fparams.radius/2.f, fparams.radius/2.f);
        }
        frects.push_back(temp);
    }

    //split up pointers...
    for(int i=0; i<flockNum; ++i)
    {
        if(i % 2 == 0)
        {
            pf2.push_back(&flock[i]);
        }else{
            pf.push_back(&flock[i]);
        }
    }

    std::vector<steer::Wall*> pw;
    for(auto& i : m_walls)
        pw.push_back(&i);

    std::vector<steer::SphereObstacle*> po;
    for(auto& i : m_obstacles)
        po.push_back(&i);

    //flock needs pointer to
    //container of pointers :D
    //COPIES ARE THE DEVIL!!! \m/
    for(int i=0; i<flockNum; ++i)
    {
        //again...split so that each
        //flock segment associates
        //with their own members
        if(i % 2 == 0)
        {
            flock[i].setNeighbors(&pf2);
            flock[i].setWalls(&pw);
            flock[i].setObstacles(&po);
        }else{
            flock[i].setNeighbors(&pf);
            flock[i].setWalls(&pw);
            flock[i].setObstacles(&po);
        }
    }

    /*
    ///////////////////////////////////////
    ///////////////////////////////////////
    The following are separate stand-alone
    components that exhibit each available
    steering behavior
    ///////////////////////////////////////
    ///////////////////////////////////////
    */

    //seeker!
    steer::BehaviorParameters p;
    p.radius = 10.f;
    steer::SeekComponent s(&p);
    s.setTarget(steer::Vector2(400.0, 300.0));
    SDL_Rect rect;
    rect.w = s.getBoundingRadius();
    rect.h = s.getBoundingRadius();
    //rect.setOrigin(sf::Vector2f(s.getBoundingRadius(), s.getBoundingRadius())/2.f);

    //wanderer!!
    steer::WanderComponent w(&p);
    w.setPosition(steer::Vector2(400.0, 300.0));
    SDL_Rect wrect;
    wrect.w = w.getBoundingRadius();
    wrect.h = w.getBoundingRadius();
    //wrect.setOrigin(sf::Vector2f(w.getBoundingRadius(), w.getBoundingRadius())/2.f);

    //an arriver!
    p.DecelerationTweaker = 2.f;
    steer::ArriveComponent a(&p);
    a.setTarget(steer::Vector2(400.0, 300.0));
    a.setPosition(steer::Vector2(600.0, 500.0));
    SDL_Rect arect;
    arect.w = a.getBoundingRadius();
    arect.h = a.getBoundingRadius();
    //arect.setOrigin(sf::Vector2f(a.getBoundingRadius(), a.getBoundingRadius())/2.f);

    //a pursuer!
    steer::PursuitComponent ps(&p);
    ps.setTarget(steer::Vector2(400.0, 300.0));
    ps.setPosition(steer::Vector2(100.0, 500.0));
    ps.setTargetAgent(&w);
    SDL_Rect psrect;
    psrect.w = ps.getBoundingRadius();
    psrect.h = ps.getBoundingRadius();
    //psrect.setOrigin(sf::Vector2f(ps.getBoundingRadius(), ps.getBoundingRadius())/2.f);

    //a flee-er!
    steer::FleeComponent flee(&p);
    flee.setPosition(steer::Vector2(200.0, 500.0));
    SDL_Rect fleerect;
    fleerect.w = flee.getBoundingRadius();
    fleerect.h = flee.getBoundingRadius();
    //fleerect.setOrigin(sf::Vector2f(flee.getBoundingRadius(), flee.getBoundingRadius())/2.f);

    //an evader!
    p.EvadeWeight = 1.f;
    steer::EvadeComponent ev(&p);
    ev.setPosition(steer::Vector2(200.0, 500.0));
    ev.setTargetAgent(&s);
    SDL_Rect evrect;
    evrect.w = ev.getBoundingRadius();
    evrect.h = ev.getBoundingRadius();
    //evrect.setOrigin(sf::Vector2f(ev.getBoundingRadius(), ev.getBoundingRadius())/2.f);

    //a path follower
    p.FollowPathWeight = 5.f;
    steer::PathFollowingComponent pather(&path, &p);
    SDL_Rect pathrect;
    pathrect.w = pather.getBoundingRadius();
    pathrect.h = pather.getBoundingRadius();
    //pathrect.setOrigin(sf::Vector2f(pathrect.getSize().x, pathrect.getSize().y)/2.f);
    //std::vector<sf::Vertex> drawablePath;
    //for(auto w : pathPoints)
    //{
        //drawablePath.push_back(sf::Vertex(sf::Vector2f(w.x,w.y), sf::Color::White));
    //}

    //a hider!
    p.HideWeight = 10.f;
    steer::HideComponent hc(&p);
    hc.setPosition(steer::Vector2(300.0, 500.0));
    hc.setTargetAgent(&s);
    hc.setObstacles(&po);
    SDL_Rect hcrect;
    hcrect.w = hc.getBoundingRadius();
    hcrect.h = hc.getBoundingRadius();
    //hcrect.setOrigin(sf::Vector2f(hc.getBoundingRadius(), hc.getBoundingRadius())/2.f);

    //an interposer!
    p.InterposeWeight = 50.f;
    p.MaxForce = 1000.f;
    p.MaxSpeed = 500.f;
    steer::InterposeComponent ic(&p);
    ic.setPosition(steer::Vector2(300.0, 500.0));
    ic.setAgents(&pather, &ev);
    SDL_Rect icrect;
    icrect.w = ic.getBoundingRadius();
    icrect.h = ic.getBoundingRadius();
    //icrect.setOrigin(sf::Vector2f(ic.getBoundingRadius(), ic.getBoundingRadius())/2.f);

    //an offset pursuer!
    p.OffsetPursuitWeight = 10.f;
    steer::OffsetPursuitComponent offc(&p);
    offc.setPosition(steer::Vector2(300.0, 500.0));
    offc.setLeader(&ps);
    SDL_Rect offcrect;
    offcrect.w = offc.getBoundingRadius();
    offcrect.h = offc.getBoundingRadius();
    //offcrect.setOrigin(sf::Vector2f(offc.getBoundingRadius(), offc.getBoundingRadius())/2.f);

	if( !init() )
	{
		printf( "Initialization failed!\n" );
	}
	else
	{
        bool running = true;

        SDL_Event e;

        //rect code...

        while( running )
        {
            int x,y;
            SDL_GetMouseState(&x, &y);

            while( SDL_PollEvent( &e ) != 0 )
            {
                if( e.type == SDL_QUIT )
                {
                    running = false;
                }else if(e.type == SDL_KEYUP && e.key.repeat == 0){
                   switch(e.key.keysym.sym)
                   {
                   case SDLK_ESCAPE:
                    running = false;
                    break;
                   default:
                    break;
                   }
                }
            }

            static double lastFrameTime = 0.0;
            static double cyclesLeftOver = 0.0;
            double currentTime;
            double updateIterations;

            currentTime = SDL_GetTicks();
            updateIterations = ((currentTime - lastFrameTime) + cyclesLeftOver);

            if (updateIterations > (MAX_CYCLES_PER_FRAME * UPDATE_INTERVAL)) {
                updateIterations = (MAX_CYCLES_PER_FRAME * UPDATE_INTERVAL);
            }

            while (updateIterations > UPDATE_INTERVAL) {
                updateIterations -= UPDATE_INTERVAL;

                s.Update(updateIterations);
                s.setTarget(steer::Vector2(x,y));
                WrapAround(s.m_agentPosition, WINDOWX, WINDOWY);
                w.Update(updateIterations);
                w.setTarget(steer::Vector2(x,y));
                WrapAround(w.m_agentPosition, WINDOWX, WINDOWY);
                for(int i=0; i<flockNum; ++i)
                {
                    flock[i].Update(updateIterations);

                    if(i % 2 != 0)
                        flock[i].setTarget(steer::Vector2(x,y));

                    WrapAround(flock[i].m_agentPosition, WINDOWX, WINDOWY);
                }
                a.Update(updateIterations);
                a.setTarget(steer::Vector2(x,y));
                WrapAround(a.m_agentPosition, WINDOWX, WINDOWY);
                ps.Update(updateIterations);
                WrapAround(ps.m_agentPosition, WINDOWX, WINDOWY);
                flee.Update(updateIterations);
                flee.setTarget(steer::Vector2(x,y));
                WrapAround(flee.m_agentPosition, WINDOWX, WINDOWY);
                ev.Update(updateIterations);
                WrapAround(ev.m_agentPosition, WINDOWX, WINDOWY);
                pather.Update(updateIterations);
                hc.Update(updateIterations);
                WrapAround(hc.m_agentPosition, WINDOWX, WINDOWY);
                ic.Update(updateIterations);
                WrapAround(ic.m_agentPosition, WINDOWX, WINDOWY);
                offc.Update(updateIterations);
                WrapAround(offc.m_agentPosition, WINDOWX, WINDOWY);
            }

            cyclesLeftOver = updateIterations;
            lastFrameTime = currentTime;

            SDL_SetRenderDrawColor( renderer, 0x00, 0x00, 0x00, 0xFF );
            SDL_RenderClear( renderer );

            //draw boids
            rect.x = s.getPosition().x;
            rect.y = s.getPosition().y;
            //rect.setRotation(s.getRotation());
            arect.x = a.getPosition().x;
            arect.y = a.getPosition().y;
            //arect.setRotation(a.getRotation());
            wrect.x = w.getPosition().x;
            wrect.y = w.getPosition().y;
            //wrect.setRotation(w.getRotation());
            psrect.x = ps.getPosition().x;
            psrect.y = ps.getPosition().y;
            //psrect.setRotation(ps.getRotation());
            fleerect.x = flee.getPosition().x;
            fleerect.y = flee.getPosition().y;
            //fleerect.setRotation(flee.getRotation());
            evrect.x = ev.getPosition().x;
            evrect.y = ev.getPosition().y;
            //evrect.setRotation(ev.getRotation());
            pathrect.x = pather.getPosition().x;
            pathrect.y = pather.getPosition().y;
            //pathrect.setRotation(pather.getRotation());
            hcrect.x = hc.getPosition().x;
            hcrect.y = hc.getPosition().y;
            //hcrect.setRotation(hc.getRotation());
            icrect.x = ic.getPosition().x;
            icrect.y = ic.getPosition().y;
            //icrect.setRotation(ic.getRotation());
            offcrect.x = offc.getPosition().x;
            offcrect.y = offc.getPosition().y;
            //offcrect.setRotation(offc.getRotation());

            SDL_SetRenderDrawColor(renderer, 0xFF, 0x00, 0x00, 0xFF);
			SDL_RenderFillRect(renderer, &rect);
            SDL_SetRenderDrawColor(renderer, 0xFF, 0xFF, 0xFF, 0xFF);
			SDL_RenderFillRect(renderer, &wrect);
            SDL_SetRenderDrawColor(renderer, 0xFF, 0x00, 0xFF, 0xFF);
			SDL_RenderFillRect(renderer, &arect);
            SDL_SetRenderDrawColor(renderer, 0x99, 0xFF, 0xFF, 0xFF);
			SDL_RenderFillRect(renderer, &psrect);
            SDL_SetRenderDrawColor(renderer, 0xFF, 0xFF, 0x00, 0xFF);
			SDL_RenderFillRect(renderer, &fleerect);
			SDL_SetRenderDrawColor(renderer, 0xFF, 0x00, 0xFF, 0xFF);
			SDL_RenderFillRect(renderer, &evrect);
			SDL_SetRenderDrawColor(renderer, 0xFF, 0x99, 0x99, 0xFF);
			SDL_RenderFillRect(renderer, &pathrect);
            SDL_SetRenderDrawColor(renderer, 0x99, 0x99, 0xFF, 0xFF);
			SDL_RenderFillRect(renderer, &hcrect);
            SDL_SetRenderDrawColor(renderer, 0x99, 0xFF, 0x99, 0xFF);
			SDL_RenderFillRect(renderer, &icrect);
			SDL_SetRenderDrawColor(renderer, 0xCC, 0xCC, 0xFF, 0xFF);
			SDL_RenderFillRect(renderer, &offcrect);

            for(int i=0; i<flockNum; ++i)
            {
                frects[i].x = flock[i].getPosition().x;
                frects[i].y = flock[i].getPosition().y;
                //frects[i].setRotation(flock[i].getRotation());
                if(i % 2 == 0){
                    SDL_SetRenderDrawColor(renderer, 0xFF, 0x99, 0x00, 0xFF);
                    SDL_RenderFillRect(renderer, &frects[i]);
                }else{
                    SDL_SetRenderDrawColor(renderer, 0x99, 0xFF, 0x99, 0xFF);
                    SDL_RenderFillRect(renderer, &frects[i]);
                }
            }
            //sf::CircleShape temp;
            //for(auto i : m_obstacles)
            //{
                //temp.setFillColor(sf::Color(0.f,0.f,0.f,0.f));
                //temp.setOutlineThickness(2.f);
                //temp.setOutlineColor(sf::Color(0,255,255,255));
                //temp.setPosition(sf::Vector2f(i.getPosition().x, i.getPosition().y));
                //temp.setRadius(i.getRadius());
                //temp.setOrigin(temp.getRadius(),temp.getRadius());
                //window.draw(temp);
            //}
            //for(auto i : m_walls)
            //{
                //sf::VertexArray lines(sf::Lines, 2);
                //lines[0].color = sf::Color(255,255,255,255);
                //lines[0].position = sf::Vector2f(i.From().x, i.From().y);
                //lines[1].color = sf::Color(255,255,255,255);
                //lines[1].position = sf::Vector2f(i.To().x, i.To().y);
                //window.draw(lines);
            //}

            //window.draw(&drawablePath[0], drawablePath.size(), sf::LineStrip);

            //Update screen
            SDL_RenderPresent( renderer );
        }
	}

	return 0;
}

bool init()
{
    //Create window
    window = SDL_CreateWindow( "SDL Tutorial", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, WINDOWX, WINDOWY, SDL_WINDOW_SHOWN );
    if( window == NULL )
    {
        printf( "Window could not be created! SDL Error: %s\n", SDL_GetError() );
        return false;
    }
    else
    {
        //Create vsynced renderer for window
        renderer = SDL_CreateRenderer( window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC );
        if( renderer == NULL )
        {
            printf( "Renderer could not be created! SDL Error: %s\n", SDL_GetError() );
            return false;
        }
        else
        {
            //Initialize renderer color
            SDL_SetRenderDrawColor( renderer, 0xFF, 0xFF, 0xFF, 0xFF );
            return true;
        }
    }
}

void createWalls()
{
    //create the walls
    const int NumWallVerts = 4;

    steer::Vector2 walls[NumWallVerts] = {
        steer::Vector2(WALLPADDING, WALLPADDING),
        steer::Vector2(WINDOWX - WALLPADDING, WALLPADDING),
        steer::Vector2(WINDOWX - WALLPADDING, WINDOWY - (WALLPADDING*3)),
        steer::Vector2(WALLPADDING, WINDOWY - (WALLPADDING*3))
    };

    for (int w=0; w<NumWallVerts-1; ++w)
    {
        m_walls.push_back(steer::Wall(true, walls[w], walls[w+1]));
    }

    m_walls.push_back(steer::Wall(true, walls[NumWallVerts-1], walls[0]));
}

void createObstacles()
{
    for (int o=0; o<OBSTACLES; ++o)
    {
        bool bOverlapped = true;

        int NumTrys = 0;
        int NumAllowableTrys = MAXITERS;

        while (bOverlapped)
        {
            NumTrys++;

            if (NumTrys > NumAllowableTrys) return;

            int radius = steer::RandInt(MINRADIUS, MAXRADIUS);

            const int border                 = MINBORDER;
            const int MinGapBetweenObstacles = MINGAP;

            steer::SphereObstacle ob = steer::SphereObstacle(steer::Vector2(steer::RandInt(radius+border, WINDOWX-radius-border),steer::RandInt(radius+border, WINDOWY-radius-30-border)), radius);

            if (!steer::Overlapped(&ob, m_obstacles, MinGapBetweenObstacles))
            {
                m_obstacles.push_back(ob);
                bOverlapped = false;
            }
        }
    }
}
