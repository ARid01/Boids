#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <string>

float maximumSpeed = 4.0f;
float maximumForce = 0.1f;
int winW = 800;
int winH = 800;
int nBoids = 100;
int nPreds = 3;
float desiredSeparation = 25.0f;
float aliNeighborDist = 50.0f;
float cohNeighborDist = 50.0f;
float avoidMag = 1.0f;
float sepDist = 50.0f;
float sepMod = 1.5f;
int boidSize = 4;
int predSize = 7;
float safeDist = 100.0f;
float huntRange = 90.0f;
float eatDist = 10.0f;
int velInitRange = 100;
float addTime = 0.125f;
int maxLen = 20;
float edgeDst = 10.0f;
float edgeForce = 20.0f;
float starveTime = 8.0f;
bool trail = true;

float magnitude(const sf::Vector2f& v) {
	return std::sqrtf(v.x * v.x + v.y * v.y);
}

class Boid {
public:
	sf::Vector2f pos, vel, acc;
	float maxSpeed, maxForce;
	sf::Color color;
	bool rotDir;
	sf::VertexArray va;
	sf::Clock clock;
	bool isPred;
	sf::Clock hunger;

	Boid(float x, float y, sf::Color c, bool pred = false) {
		isPred = pred;
		pos = sf::Vector2f(x, y);
		vel = sf::Vector2f(static_cast<float>((std::rand() % velInitRange - (velInitRange / 2.0f)) / (velInitRange / 2.0f)),
			static_cast<float>((std::rand() % velInitRange - (velInitRange / 2.0f)) / (velInitRange / 2.0f)));
		acc = sf::Vector2f(0.0f, 0.0f);
		maxSpeed = maximumSpeed;
		maxForce = maximumForce;
		color = c;
		rotDir = (bool)(rand() % 2);
		va = sf::VertexArray(sf::LineStrip);
		va.append(sf::Vertex(pos, sf::Color(color.r, color.g, color.b, 100)));
		clock.restart();
		if (pred) hunger.restart();
	}

	void update() {
		vel += acc;
		if (magnitude(vel) > maxSpeed) {
			vel = setMagnitude(vel, maxSpeed);
		}
		pos += vel;
		acc *= 0.0f;

		if (clock.getElapsedTime().asSeconds() > addTime && trail) {
			clock.restart();
			va.append(sf::Vertex(pos, sf::Color(color.r, color.g, color.b, 100)));

			if (va.getVertexCount() > maxLen) {
				int del = va.getVertexCount() - maxLen;

				std::vector<sf::Vertex> v;
				for (int i = del; i < va.getVertexCount(); i++) {
					v.push_back(va[i]);
				}
				va.clear();
				
				for (sf::Vertex& vt : v) {
					va.append(vt);
				}
			}
		}
	}

	void applyForce(sf::Vector2f force) {
		acc += force;
	}

	sf::Vector2f separate(const std::vector<Boid> &boids, float desiredSeparation) {
		sf::Vector2f steer(0.0f, 0.0f);
		int count = 0;
		for (const auto& other : boids) {
			float d = magnitude(pos - other.pos);
			if (d > 0 && d < desiredSeparation) {
				sf::Vector2f diff = pos - other.pos;
				diff = setMagnitude(diff, 1.0f);
				diff /= d;
				steer += diff;
				count++;
			}
		}

		if (count > 0)
			steer /= static_cast<float>(count);

		if (magnitude(steer) > 0) {
			steer = setMagnitude(steer, maxSpeed);
			steer -= vel;
			if (magnitude(steer) > maxForce)
				steer = setMagnitude(steer, maxForce);
		}
		return steer;
	}

	sf::Vector2f avoid(const sf::Vector2i& av, float sepDist) {
		sf::Vector2f nav = sf::Vector2f(av.x, av.y);
		sf::Vector2f steer(0.0f, 0.0f);
		float d = magnitude(pos - nav);
		if (d < sepDist) {
			sf::Vector2f dif = pos - nav;
			dif = setMagnitude(dif, 1.0f);
			steer += (rotDir ? sf::Vector2f(-dif.y, dif.x) : sf::Vector2f(dif.y, -dif.x)) * avoidMag;
		}
		return steer;
	}

	sf::Vector2f bounds(float dst, float boundaryForce) {
		sf::Vector2f force(0.0f, 0.0f);

		if (pos.x < dst)
			force.x = boundaryForce;
		else if (pos.x > winW - dst)
			force.x = -boundaryForce;

		if (pos.y < dst)
			force.y = boundaryForce;
		else if (pos.y > winH - dst)
			force.y = -boundaryForce;

		return force;
	}

	sf::Vector2f align(const std::vector<Boid>& boids, float neighborDist) {
		sf::Vector2f sum(0.0f, 0.0f);
		int count = 0;
		for (const auto& other : boids) {
			float d = magnitude(pos - other.pos);
			if (d > 0 && d < neighborDist) {
				sum += other.vel;
				count++;
			}
		}
		if (count > 0) {
			sum /= static_cast<float>(count);
			sum = setMagnitude(sum, maxSpeed);
			sf::Vector2f steer = sum - vel;
			if (magnitude(steer) > maxForce) {
				steer = setMagnitude(steer, maxForce);
			}
			return steer;
		}
		return sf::Vector2f(0.0f, 0.0f);
	}

	bool isStarved() {
		return (hunger.getElapsedTime().asSeconds() > starveTime);
	}

	sf::Vector2f cohesion(const std::vector<Boid>& boids, float neighborDist) {
		sf::Vector2f sum(0.0f, 0.0f);
		int count = 0;
		for (const auto& other : boids) {
			float d = magnitude(pos - other.pos);
			if (d > 0 && d < neighborDist) {
				sum += other.pos;
				count++;
			}
		}

		if (count > 0) {
			sum /= static_cast<float>(count);
			return seek(sum);
		}

		return sf::Vector2f(0.0f, 0.0f);
	}

	sf::Vector2f seek(sf::Vector2f target) {
		sf::Vector2f desired = target - pos;
		desired = setMagnitude(desired, maxSpeed);
		sf::Vector2f steer = desired - vel;
		if (magnitude(steer) > maxForce)
			steer = setMagnitude(steer, maxForce);
		return steer;
	}

	sf::Vector2f avoidPred(const std::vector<Boid>& predators, float safeDist) {
		sf::Vector2f steer(0.0f, 0.0f);
		int count = 0;

		for (const auto& pred : predators) {
			float d = magnitude(pos - pred.pos);
			if (d < safeDist) {
				sf::Vector2f diff = pos - pred.pos;
				diff = setMagnitude(diff, 1.0f) / d;
				steer += diff;
				count++;
			}
		}

		if (count > 0) {
			steer /= (float)count;
			steer = setMagnitude(steer, maxSpeed);
			steer -= vel;
			if (magnitude(steer) > maxForce) steer = setMagnitude(steer, maxForce);
		}
		return steer;
	}

	sf::Vector2f chasePrey(const std::vector<Boid>& prey, float huntRange) {
		Boid* closestPrey = nullptr;
		float minDist = huntRange;
		for (const auto& p : prey) {
			float d = magnitude(pos - p.pos);
			if (d < minDist) {
				minDist = d;
				closestPrey = const_cast<Boid*>(&p);
			}
		}

		if (closestPrey) return seek(closestPrey->pos);
		return sf::Vector2f(0.0f, 0.0f);
	}

	sf::Vector2f wander() {
		static float angle = 0;
		float wanderStrength = 0.4f;
		float circleDist = 2.0f;
		float circleRadius = 6.0f;

		angle += ((std::rand() % 100 - 50) / 100.0f) * wanderStrength;
		sf::Vector2f displacement(cosf(angle) * circleRadius, sinf(angle) * circleRadius);

		sf::Vector2f circleCenter = setMagnitude(vel, circleDist);
		sf::Vector2f wanderForce = circleCenter + displacement;

		return setMagnitude(wanderForce, maxForce);
	}

private:

	sf::Vector2f setMagnitude(const sf::Vector2f& v, float m) {
		float current = magnitude(v);
		if (current == 0) return v;
		return v * (m / current);
	}
};

float CrossProduct(const sf::Vector2f& a, const sf::Vector2f& b) {
	return a.x * b.y - a.y * b.x;
}

void initBoids(std::vector<Boid>& boids, int numBoids, std::vector<Boid>& preds, int numPreds) {
	boids.clear();
	preds.clear();
	for (int i = 0; i < numBoids; i++) {
		boids.push_back(Boid(std::rand() % winW, std::rand() % winH, sf::Color(rand() % 255, rand() % 255, rand() % 255)));
	}
	for (int i = 0; i < numPreds; i++) {
		preds.push_back(Boid(std::rand() % winW, std::rand() % winH, sf::Color::Red, true));
	}
}

int main() {
	std::srand(static_cast<unsigned int>(std::time(nullptr)));

	sf::RenderWindow window(sf::VideoMode(winW, winH), "Boids");
	window.setFramerateLimit(60);

	sf::Font font;
	sf::Text text;
	if (!font.loadFromFile("Minecraft.ttf")) {
		std::cout << "Failed to load font!" << std::endl;
	}
	text.setFont(font);
	text.setCharacterSize(18);
	text.setOutlineThickness(2);
	text.setPosition(sf::Vector2f(10, 10));

	std::vector<Boid> boids;
	std::vector<Boid> preds;
	const int numBoids = nBoids;
	const int numPreds = nPreds;
	initBoids(boids, numBoids, preds, numPreds);

	while (window.isOpen()) {
		sf::Event event;
		while (window.pollEvent(event)) {
			if (event.type == sf::Event::Closed) {
				window.close();
			}
			if (event.type == sf::Event::KeyPressed) {
				if (event.key.code == sf::Keyboard::Escape) {
					window.close();
				}
				if (event.key.code == sf::Keyboard::B) {
					for (int i = 0; i < 10; i++)
						boids.push_back(Boid(std::rand() % winW, std::rand() % winH, sf::Color(rand() % 255, rand() % 255, rand() % 255)));
				}
				if (event.key.code == sf::Keyboard::P) {
					preds.push_back(Boid(std::rand() % winW, std::rand() % winH, sf::Color::Red, true));
				}
				if (event.key.code == sf::Keyboard::Space) {
					initBoids(boids, numBoids, preds, numPreds);
				}
				if (event.key.code == sf::Keyboard::T) {
					trail = !trail;

					if (trail) {
						for (Boid& b : boids) {
							b.va.clear();
						}
						for (Boid& b : preds) {
							b.va.clear();
						}
					}
				}
			}
		}

		for (auto& boid : boids) {
			sf::Vector2f sep = boid.separate(boids, desiredSeparation);
			sf::Vector2f ali = boid.align(boids, aliNeighborDist);
			sf::Vector2f coh = boid.cohesion(boids, cohNeighborDist);
			sf::Vector2f msAv = boid.avoid(sf::Mouse::getPosition(window), sepDist);
			sf::Vector2f edge = boid.bounds(edgeDst, edgeForce);
			sf::Vector2f avoid = boid.avoidPred(preds, safeDist);

			boid.applyForce(sep * sepMod);
			boid.applyForce(ali);
			boid.applyForce(coh);
			boid.applyForce(msAv);
			boid.applyForce(edge);
			boid.applyForce(avoid * 2.0f);
			boid.update();

			if (boid.pos.x < 0) boid.pos.x += winW;
			if (boid.pos.y < 0) boid.pos.y += winH;
			if (boid.pos.x > winW) boid.pos.x -= winW;
			if (boid.pos.y > winH) boid.pos.y -= winH;
		}

		for (auto& pred : preds) {
			sf::Vector2f chase = pred.chasePrey(boids, huntRange);
			sf::Vector2f edge = pred.bounds(edgeDst, edgeForce);

			if (magnitude(chase) > 0)
				pred.applyForce(chase);
			else {
				sf::Vector2f wander = pred.wander();
				pred.applyForce(wander * 0.8f);
			}

			pred.applyForce(edge);
			pred.update();
		}

		for (auto it = boids.begin(); it != boids.end();) {
			bool eaten = false;
			for (auto& pred : preds) {
				if (magnitude(pred.pos - it->pos) < eatDist) {
					it = boids.erase(it);
					eaten = true;
					pred.hunger.restart();
					break;
				}
			}

			if (!eaten) it++;
		}

		for (auto it = preds.begin(); it != preds.end(); it++) {
			if (it->isStarved()) {
				preds.erase(it);
				break;
			}
		}

		//Update text
		std::string stats = "Boids (B): " + std::to_string(boids.size()) + "\nPreds (P): " + std::to_string(preds.size()) +
			"\nTrails (T): " + (trail ? "On" : "Off") + "\nRestart -> Space";
		text.setString(stats);

		window.clear();

		for (const auto& boid : boids) {
			sf::CircleShape shape(boidSize, 3);
			shape.setOrigin(sf::Vector2f(shape.getRadius(), shape.getRadius()));
			shape.setRotation(atan2f(boid.vel.y, boid.vel.x) * 180.0f / 3.14159f - 0.78f);
			shape.setPosition(boid.pos);
			shape.setFillColor(boid.color);
			if (trail)
				window.draw(boid.va);
			window.draw(shape);
		}

		for (const auto& pred : preds) {
			sf::CircleShape shape(predSize, 3);
			shape.setOrigin(sf::Vector2f(shape.getRadius(), shape.getRadius()));
			shape.setRotation(atan2f(pred.vel.y, pred.vel.x) * 180.0f / 3.14159f - 0.78f);
			shape.setPosition(pred.pos);
			shape.setFillColor(pred.color);
			if (trail)
				window.draw(pred.va);
			window.draw(shape);
		}

		window.draw(text);
		window.display();
	}

	return 0;
}