%% Plot results to metareasoning.slx

figure(1)
for i = 1:num_ac
    plot3(out.x.data,out.y.data,out.h.data)
    hold on 
    grid on
    axis equal
end
xlabel('North'), ylabel('East'), zlabel('Up')
plot3(pos_ic(1,1),pos_ic(1,2),pos_ic(1,3), 'x')
plot3(pos_ic(2,1),pos_ic(2,2),pos_ic(2,3), 'x')