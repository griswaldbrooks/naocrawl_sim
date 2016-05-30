% Quick script to check torque tables generated at different simulation speeds

thetas = [2,3,4,5];

for i = 1:size(vrep1TorqueTable2,1)

	figure(1)
	width1 = 0.5;
	bar(thetas,vrep1TorqueTable2(i,9:12), width1,'FaceColor',[0.2,0.2,0.5]);

	hold on
	width2 = width1/2;
	bar(thetas,vrepnTorqueTablem(i,9:12),width2,'FaceColor',[0,0.7,0.7],...
	                     'EdgeColor',[0,0.7,0.7])
	hold off

	legend('Real-time','Fast') % add legend

	input('Wait');
end