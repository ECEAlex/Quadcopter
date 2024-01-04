%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  Code modified by Francisco Vasconcelos from
%%%%
%%%%  Visualisation code for quadcopter 
%%%%  Author: Daniel Butters
%%%%  Date: 16/11/17
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [] = drawDrone(X,T,pos_start)

    %Define total width, length and height of flight arena (metres)
    spaceDim = 8;
    spaceLimits = [-spaceDim/2 spaceDim/2 -spaceDim/2 spaceDim/2 0 spaceDim/2];
    
    
    %figure to display drone simulation
    f1 = figure;
    ax1 = gca;
    view(ax1, 3);
    axis equal;
    axis(spaceLimits)
    grid ON
    grid MINOR
    caxis(ax1, [0 spaceDim]);
    hold(ax1,'on')
    axis vis3d
    
    num_drones = 1;
    
    %instantiate a drone object, input the axis and arena limits
    drones = [];
    for i = 1:num_drones
        drones = [drones Drone(ax1,spaceDim,num_drones,pos_start)];
    end
    
    for t = 1:(length(T)-1)
        %clear axis
        cla(ax1);
        
        %update and draw drones
        for i = 1:num_drones
            update(drones(i), X(:,t));
        end
      
        %apply fancy lighting (optional)
        camlight

        %update figure
        drawnow
    end

end
