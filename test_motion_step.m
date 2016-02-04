wall_list = { [-3 -3; -3 3], [-3 -3 ; 3 -3], [-3 3; 3 3], [3 3; 3 -3]};

corners = [-3 -3; -3 3; 3 3; 3 -3];

mot.p = [0 0 0];
mot.v = [0 0.2 0];
mot.w = 0;

dev.p_g = 0;
dev.d_g = 0;
dev.g = [0 0 0];
dev.d_a = 0;
dev.a = [0 0 0];

world.walls = wall_list;
world.corners = corners;
world.wind = [0 0 0];

dt = 0.1;
f = figure;
for i = 1:dt:10,    
    clf(f);
    hold on;
    for j = 1:length(wall_list)
        wall = wall_list{j};
        plot(wall(:,1), wall(:,2), 'b-');
    end
    new_mot = step_motion(dt, mot, dev, world);
    plot(mot.p(1), mot.p(2), 'r+');
    mot = new_mot;
    drawnow;
end