clc;clear;

grid =  xlsread('occupancy_grid.xlsx') ;

x_pos = 3.8 ;
y_pos = 1.1 ;

x_goal = 3 ;
y_goal = 4.6 ;

i = 1 ;

while x_pos ~= x_goal && y_pos ~= y_goal
    
    graph_x(i) =x_pos;
    graph_y(i) =y_pos;
    
    y_p_1 = y_pos +1;
    y_m_1 = y_pos -1;

    x_p_1 = x_pos +1;
    x_m_1 = x_pos -1;

    for i = (-1+x_pos*10 : 2+x_pos*10)
        for j = (-1+y_pos*10 : 2+y_pos*10)
            if grid(i, j) == 1
                G_fix(i, j) = inf ;
                
            end
        end
    end
    
    G_cost = [ 1.4 1.0 1.4  ;
               1.0 inf 1.0  ;
               1.4 1.0 1.4 ];

    H_cost = [ sqrt((x_goal -x_m_1)^2 + (y_goal -y_m_1)^2) sqrt((x_goal -x_pos)^2 + (y_goal -y_m_1)^2) sqrt((x_goal -x_m_1)^2 + (y_goal -y_m_1)^2)  ;
               sqrt((x_goal -x_m_1)^2 + (y_goal -y_pos)^2) sqrt((x_goal -x_pos)^2 + (y_goal -y_pos)^2) sqrt((x_goal -x_m_1)^2 + (y_goal -y_pos)^2)  ;
               sqrt((x_goal -x_m_1)^2 + (y_goal -y_p_1)^2) sqrt((x_goal -x_pos)^2 + (y_goal -y_p_1)^2) sqrt((x_goal -x_m_1)^2 + (y_goal -y_p_1)^2) ];

    F_cost = G_cost + H_cost + G_fix ;

    minimum = min(min(F_cost));
    [y, x]=find(F_cost==minimum) ;
    
    if x == 1
        x_pos = x_pos -1;
    
    elseif x == 3
        x_pos = x_pos +1;
    
    end
    
    if y == 1
        y_pos = y_pos -1;

    elseif y == 3
        y_pos = y_pos +1;
        
    end
    
    i = i +1;
    
    G_fix = [ 0 0 0  ;
               0 0 0  ;
               0 0 0 ];
end


