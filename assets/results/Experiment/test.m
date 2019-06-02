load trial1.mat

figure(1)
axis equal; hold on

localizedSC = [0.0, 0.0; 1.7679467693696225, -2.1855250205497256; -0.898089838690996, -2.1717859066170577; -5.1220536354472905, -1.6947873837375036; -7.876657768662047, -0.4728141413387805; -9.270912114115992, 1.6660125753957844; -8.226627737228872, 2.2456752644225153; -11.900225048785426, 0.9705937299062669; -14.821044579397721, 2.4274916959577286; -16.353457071472043, 4.136681933907432; -19.057111134690768, 5.343452295677629; -21.770754033735333, 8.34271240565397; -23.108886717378233, 11.104257559525898; -19.73226931039429, 12.110252887541698; -22.157140911633356, 12.223832159942015; -24.522279348072466, 11.35950505243939; -26.60976423245829, 14.73564071369609; -28.9390450408091, 18.49626054098901; -29.571882686348722, 21.84259223086911; -31.457886240124953, 24.76449585042315; -31.71142492955367, 28.070877055737533; -33.35428061507479, 31.114858218381432; -33.29002460382074, 33.96906364994246; -34.435213387841415, 36.34065866348211; -34.360420722704134, 39.704833643843756; -35.45017739926526, 43.07773740741544; -36.922512375650754, 44.638413985618485; -33.297716409878454, 44.23285416115842; -30.89689929982731, 43.66058736159268; -35.57588580302238, 44.24599961183527; -35.84763237454805, 40.68496001800661; -37.85305789398504, 37.01220933349822; -38.085370145865014, 33.587998998379774; -39.82043580341343, 31.277268171966945; -41.538572191405954, 25.519754888541392; -43.9664097605589, 22.63657810847017; -46.31860435968171, 16.522082870442592; -49.00646389614047, 12.30808082440889; -46.378837525177765, 9.996675053480574; -46.09602021587155, 8.198505604554231; -47.55951784831997, 11.487160646714942; -52.42329941003783, 8.767496492163803; -56.304500523443146, 6.282611073433732; -60.92546041242433, 1.5610819726285536; -64.77804292103549, -0.07024393806043479; -68.73037295588216, -3.4789129580701887; -71.67860692225904, -4.600345931076915; -68.52770686723997, -6.508695173204873];
localizedHC = [0.0, 0.0; 1.7697281103940778, -2.198409775257787; -0.9205489780142938, -2.225781326250232; -4.624262801313323, -1.5587360192257724; -7.659018728139029, -0.47135904380942717; -9.416297411790628, 1.644833427626512; -8.30593272774369, 2.2319680687996635; -11.892940176640602, 0.9578169633150901; -14.701828129316219, 2.4159363095096498; -16.141892261581855, 4.082106535915406; -18.643905977336637, 5.219543891507912; -21.26305043256835, 8.148795711154651; -22.849222562451647, 10.942112439727827; -19.112540804173147, 11.807390365438438; -21.77980314452206, 12.027408834970275; -23.40040207219697, 10.892440508138387; -26.80667082004809, 14.81721711759769; -28.492016805674524, 18.228789318636597; -29.389934320291733, 21.775640370871418; -30.868173549122766, 24.415076127217546; -31.208379151870034, 27.736705467067583; -32.330017893358914, 30.47974967451581; -31.61788337464569, 32.71191909500289; -33.08730194196812, 34.95877948997366; -30.753524715812322, 36.36740258705951; -34.05424254443175, 42.03300015829883; -34.09883084555691, 41.83553926451058; -30.588964416788798, 41.41317947403538; -29.13039785061743, 41.542968485225124; -31.086811256615587, 43.586275991113574; -31.024803378193624, 36.26591316675694; -34.08372408210016, 34.42620465240892; -34.322649858226804, 30.512460293358476; -33.40803625291151, 27.517912965694737; -34.60221862057235, 22.19887551936125; -38.88185658519724, 19.555533831800307; -42.24409236578938, 14.406566028185727; -42.9482722382488, 10.75813598382772; -42.033939971003065, 8.440158701504195; -41.34369245159356, 6.746192544933467; -42.164343865151274, 10.238353873942998; -43.6909802415263, 8.334050408916891; -50.425412374817995, 4.777367251889923; -55.56887703020392, 0.2033482886717921; -59.43433292299938, -1.4161069892640064; -63.35794006501541, -4.782957951337764; -66.3320315363386, -5.899483451275247; -63.1543654408985, -7.846552831698994];


plot(groundtruth(:,1), groundtruth(:,2), 'k-o')
plot(odom(:,1), odom(:,2), 'r-o')
plot(localizedNC(:,1), localizedNC(:,2), 'g--x')
plot(localizedSC(:,1), localizedSC(:,2), 'c--x')
plot(localizedHC(:,1), localizedHC(:,2), 'b--.')

legend('GroundTruth', 'Odometry', 'Path NC', 'Path SC', 'Path HC')

fprintf("MSE in the ground truth and odom is %f\n",calcMSE(groundtruth, odom));
fprintf("MSE in the ground truth and localization NC is %f\n",calcMSE(groundtruth, localizedNC));
fprintf("MSE in the ground truth and localization SC is %f\n",calcMSE(groundtruth, localizedSC));
fprintf("MSE in the ground truth and localization HC is %f\n",calcMSE(groundtruth, localizedHC));


function mse = calcMSE(a, b)
    mse = 0; n = length(a);
    for i=1:n
       mse = mse + sqrt( (a(i,1) - b(i,1))^2 + (a(i,2) - b(i,2))^2);
    end
    mse = mse/n;
end