% Quick script to check torque tables generated at different simulation speeds

thetas = [2,3,4,5];

for i = 1:size(vrepnTorqueTablem,1)

	figure(1)
	width1 = 0.5;
	bar(thetas,vrepnTorqueTablem(i,9:12),width1,'FaceColor',[0,0.7,0.7],...
	                     'EdgeColor',[0,0.7,0.7])

	legend('2.5 degree') % add legend
	disp(['Iteration: ', num2str(i)]);
	% input('Wait');
end