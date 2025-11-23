t = linspace(0,5,1000);
ang = deg2rad(20 * (1 - exp(-t / 0.5)));

plot(t, ang)