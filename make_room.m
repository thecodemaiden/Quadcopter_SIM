function [theRoom] = make_room(center_x, center_y, center_z, w, l, h)

[X, Y, Z] = prism_faces(center_x, center_y, center_z, w, l,h);
corners = [X', Y', Z']; %8 rows, 3 columns
edges= cell(12,1);
edge_list = {[1,2], [1,3], [1,5], [2,4], [2,6], [3,4], [3,7], [4,8],...
             [5,6], [5,7], [6,8], [7,8]};
for i=1:12
    edges{i} = corners(edge_list{i},:);
end

faces = cell(6,1);
face_list = {[1,2,4,3], [1,2,6,5], [1,3,7,5], [5,6,8,7], [3,4,8,7],...
             [2,4,8,6]};
for i=1:6
    faces{i} = corners(face_list{i},:);
end
theRoom.corners = corners;
theRoom.edges = edges;
theRoom.faces = faces;
end