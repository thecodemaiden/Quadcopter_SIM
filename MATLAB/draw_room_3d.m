function draw_room_3d(aRoom, fig)
 edges = aRoom.edges;
 figure(fig);
 for i=1:12,
     e = edges{i};
     plot3(e(:,1), e(:,2), e(:,3), 'Color', [0, 0.4, 0], 'LineWidth', 2);
 end