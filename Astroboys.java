package astroboys;

import robocode.*;
import robocode.util.Utils;
import java.awt.Color;
import java.awt.geom.Point2D;

/**
 * Astroboys - Robocode (Java 8)
 * Compatível com Robocode/Java 8.
 *
 * Estratégia:
 *  - Radar travado no alvo
 *  - Movimento orbital com wall-smoothing
 *  - Inverte direção quando detecta tiro inimigo
 *  - Mira guess-factor simples (histograma) + fallback circular
 *
 * Salve como: C:\robocode\robots\Astroboys\Astroboys.java
 */
public class Astroboys extends AdvancedRobot {

    // ---- Config ----
    private static final double WALL_MARGIN = 36;            // margem pra não raspar parede
    private static final double MAX_FIRE_POWER = 2.4;        // controla aquecimento
    private static final double MIN_FIRE_POWER = 0.1;
    private static final int    BINS = 31;                   // bins do GF (ímpar)

    // ---- Estado ----
    private Enemy enemy = new Enemy();
    private double moveDirection = 1;      // 1 ou -1 (invertido quando inimigo atira ou bate na parede)
    private double lastEnemyEnergy = 100;  // para detectar tiro
    private long   lastScanTime = -1;      // para manter radar vivo
    private double[] gfBins = new double[BINS]; // histograma do guess-factor

    // -------------------------------------------------------------
    // Loop principal
    // -------------------------------------------------------------
    public void run() {
        // Desacoplar gun e radar do corpo
        setAdjustGunForRobotTurn(true);
        setAdjustRadarForGunTurn(true);

        // Visual
        setColors(new Color(30, 30, 30), new Color(70, 70, 200), new Color(200, 70, 70));

        // Começa varrendo infinito
        setTurnRadarRightRadians(Double.POSITIVE_INFINITY);
        setMaxVelocity(8);

        while (true) {
            // Se o radar “parar”, manda girar infinito de novo
            if (getRadarTurnRemaining() == 0) {
                setTurnRadarRightRadians(Double.POSITIVE_INFINITY);
            }
            // Se perder o alvo por alguns ticks, dá uma mexida básica
            if (getTime() - lastScanTime > 2) {
                setTurnRightRadians(0.05 * moveDirection);
                setAhead(80 * moveDirection);
            }
            execute();
        }
    }

    // -------------------------------------------------------------
    // Evento principal: achou inimigo
    // -------------------------------------------------------------
    public void onScannedRobot(ScannedRobotEvent e) {
        lastScanTime = getTime();

        double absBearing = getHeadingRadians() + e.getBearingRadians();
        double enemyX = getX() + Math.sin(absBearing) * e.getDistance();
        double enemyY = getY() + Math.cos(absBearing) * e.getDistance();

        enemy.update(e, absBearing, enemyX, enemyY);

        // Radar lock (dobro do erro pra "colar" o radar)
        double radarTurn = Utils.normalRelativeAngle(absBearing - getRadarHeadingRadians());
        setTurnRadarRightRadians(radarTurn * 2);

        // Movimento: orbitando com wall-smoothing; inverte se detecta tiro
        if (enemyFired(e)) {
            moveDirection = -moveDirection;
        }
        doOrbitMovement(enemyX, enemyY);

        // Mira: guess-factor leve + fallback circular
        double power = chooseFirePower(e);
        double aimAngle = aimGuessFactor(absBearing, e, power, enemyX, enemyY);
        double gunTurn = Utils.normalRelativeAngle(aimAngle - getGunHeadingRadians());
        setTurnGunRightRadians(gunTurn);

        if (Math.abs(getGunTurnRemaining()) < 0.3 && getGunHeat() == 0.0) {
            setFire(power);
        }
    }

    // -------------------------------------------------------------
    // Eventos de dano/parede (movimento reativo simples)
    // -------------------------------------------------------------
    public void onHitByBullet(HitByBulletEvent e) {
        moveDirection = -moveDirection;
        setAhead(120 * moveDirection);
    }

    public void onHitWall(HitWallEvent e) {
        moveDirection = -moveDirection;
        setAhead(120 * moveDirection);
    }

	public void onBulletHit(BulletHitEvent e) {
	    registerGuessFactor(e);
	}
	
	public void onBulletMissed(BulletMissedEvent e) {
	    registerGuessFactor(e);
	}


    // -------------------------------------------------------------
    // Auxiliares de mira
    // -------------------------------------------------------------
    private double chooseFirePower(ScannedRobotEvent e) {
        double dist = e.getDistance();
        double power = Math.min(MAX_FIRE_POWER, Math.max(MIN_FIRE_POWER, 500 / dist));

        // Conserva energia quando baixo
        if (getEnergy() < 15) power = Math.min(power, 1.4);
        if (getEnergy() < 5)  power = 0.1;

        return power;
    }

    private double aimGuessFactor(double absBearing, ScannedRobotEvent e, double power,
                                  double ex, double ey) {

        // Pega bin mais “forte”
        int bestIndex = BINS / 2;
        double best = -1;
        for (int i = 0; i < BINS; i++) {
            if (gfBins[i] > best) { best = gfBins[i]; bestIndex = i; }
        }

        // Converte bin em guess-factor [-1..1]
        double guessFactor = (bestIndex - (BINS - 1) / 2.0) / ((BINS - 1) / 2.0);

        double lateralDirection = Math.signum(Math.sin(e.getHeadingRadians() - absBearing));
        double escapeAngle = Math.asin(8.0 / bulletSpeed(power));
        double gfAngle = absBearing + lateralDirection * guessFactor * escapeAngle;

        // Fallback “circular” (se histograma vazio)
        if (best <= 0.0001) {
            double deltaH = e.getHeadingRadians() - enemy.lastHeading;
            double bulletTime = e.getDistance() / bulletSpeed(power);
            double futureHeading = e.getHeadingRadians() + deltaH * bulletTime;
            double vx = Math.sin(futureHeading) * e.getVelocity();
            double vy = Math.cos(futureHeading) * e.getVelocity();
            // simples aproximação linear
            double fx = ex + vx * bulletTime;
            double fy = ey + vy * bulletTime;
            gfAngle = Math.atan2(fx - getX(), fy - getY());
        }

        return gfAngle;
    }

	private void registerGuessFactor(BulletHitEvent e) {
	    updateGuessFactor(e.getBullet().getPower(), true);
	}
	
	private void registerGuessFactor(BulletMissedEvent e) {
	    updateGuessFactor(e.getBullet().getPower(), false);
	}

	private void updateGuessFactor(double bulletPower, boolean hit) {
	    double absBearingAtFire = enemy.lastAbsBearing;
	    double enemyHeadingAtFire = enemy.lastHeading;
	    double lateralDirection = Math.signum(Math.sin(enemyHeadingAtFire - absBearingAtFire));
	
	    double bearingNow = Math.atan2(enemy.x - getX(), enemy.y - getY());
	    double offset = Utils.normalRelativeAngle(bearingNow - absBearingAtFire);
	
	    double escapeAngle = Math.asin(8.0 / bulletSpeed(bulletPower));
	    double gf = offset / escapeAngle;
	    gf *= lateralDirection;
	    if (gf < -1) gf = -1;
	    if (gf >  1) gf =  1;
	
	    int index = (int) Math.round((gf + 1) * (BINS - 1) / 2.0);
	    if (index < 0) index = 0;
	    if (index >= BINS) index = BINS - 1;
	
	    gfBins[index] += hit ? 2.0 : 0.5;
	}


    private double bulletSpeed(double power) {
        return 20.0 - 3.0 * power;
    }

    // -------------------------------------------------------------
    // Auxiliares de movimento
    // -------------------------------------------------------------
    private boolean enemyFired(ScannedRobotEvent e) {
        boolean fired = e.getEnergy() < lastEnemyEnergy && lastEnemyEnergy - e.getEnergy() <= 3.01;
        lastEnemyEnergy = e.getEnergy();
        return fired;
    }

    private void doOrbitMovement(double targetX, double targetY) {
        // ângulo desejado = perpendicular ao inimigo
        double desired = Math.atan2(targetX - getX(), targetY - getY()) + moveDirection * Math.PI / 2;

        // wall-smoothing por iteração
        desired = wallSmoothing(getX(), getY(), desired, moveDirection);

        double turn = Utils.normalRelativeAngle(desired - getHeadingRadians());
        setTurnRightRadians(turn);
        setAhead(100 * moveDirection);
    }

    private double wallSmoothing(double x, double y, double angle, double orientation) {
        double stick = 120;
        for (int i = 0; i < 20; i++) {
            double testX = x + Math.sin(angle) * stick;
            double testY = y + Math.cos(angle) * stick;
            if (testX > WALL_MARGIN && testY > WALL_MARGIN &&
                testX < getBattleFieldWidth() - WALL_MARGIN &&
                testY < getBattleFieldHeight() - WALL_MARGIN) {
                break;
            }
            angle += orientation * 0.1;
        }
        return angle;
    }

    // -------------------------------------------------------------
    // Estado do inimigo
    // -------------------------------------------------------------
    private static class Enemy {
        String name;
        double x, y;
        double lastAbsBearing;
        double lastHeading;

        void update(ScannedRobotEvent e, double absBearing, double ex, double ey) {
            name = e.getName();
            x = ex; y = ey;
            lastAbsBearing = absBearing;
            lastHeading = e.getHeadingRadians();
        }
    }
}


