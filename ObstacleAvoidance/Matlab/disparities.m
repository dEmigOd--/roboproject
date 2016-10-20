filename = 'RobotFov';
leftname = 'Left';
rightname = 'Right';

left = imread(sprintf('../images/%s_%s_%05d.png', filename, leftname, 1));
right = imread(sprintf('../images/%s_%s_%05d.png', filename, rightname, 1));

subplot(1, 2, 2);
imshow(left);
subplot(1, 2, 1);
imshow(right);

hold on
plot(100,80,'or','markersize',4,'linewidth',1)
