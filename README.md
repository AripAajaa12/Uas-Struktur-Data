import random import itertools import heapq import matplotlib.pyplot as plt import networkx as nx
import random
import itertools
import heapq
import matplotlib.pyplot as plt
import networkx as nx
# =======================
# 1. DEFINISI GRAPH
# =======================
class Graph:
    def __init__(self):
        self.edges = {}

    def add_edge(self, from_node, to_node, weight):
        self.edges.setdefault(from_node, []).append((to_node, weight))
        self.edges.setdefault(to_node, []).append((from_node, weight))

    def display(self):
        for node in self.edges:
            print(f"{node}: {self.edges[node]}")

# =======================
# 2. DATA KOTA
# =======================
cities = ["Pyongyang", "Nampo", "Kaesong", "Sariwon", "Wonsan", 
          "Hamhung", "Chongjin", "Hyesan", "Rason", "Sinuiju"]

graph = Graph()

# =======================
# 3. GENERATE GRAPH
# =======================
# Buat spanning tree dulu agar terhubung
connected = set()
connected.add(cities[0])
while len(connected) < len(cities):
    a = random.choice(list(connected))
    b = random.choice([city for city in cities if city not in connected])
    weight = random.randint(10, 100)
    graph.add_edge(a, b, weight)
    connected.add(b)

# Tambahkan edge acak hingga total 30
all_possible = list(itertools.combinations(cities, 2))
random.shuffle(all_possible)
current_edges = set()
for city in graph.edges:
    for neighbor, _ in graph.edges[city]:
        current_edges.add(tuple(sorted((city, neighbor))))

while len(current_edges) < 30:
    a, b = random.choice(all_possible)
    if tuple(sorted((a, b))) not in current_edges:
        weight = random.randint(10, 100)
        graph.add_edge(a, b, weight)
        current_edges.add(tuple(sorted((a, b))))

# =======================
# 4. VISUALISASI GRAPH
# =======================
def visualize_graph():
    G = nx.Graph()
    for city in graph.edges:
        for neighbor, weight in graph.edges[city]:
            if (neighbor, city) not in G.edges:
                G.add_edge(city, neighbor, weight=weight)

    pos = nx.spring_layout(G, seed=42)
    plt.figure(figsize=(12, 8))
    nx.draw(G, pos, with_labels=True, node_color='lightblue', node_size=2000, font_size=10, font_weight='bold')
    labels = nx.get_edge_attributes(G, 'weight')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)
    plt.title("Peta Rute Kota di Korea Utara")
    plt.show()

# =======================
# 5. DIJKSTRA
# =======================
def dijkstra(graph, start, end):
    queue = [(0, start, [])]
    visited = set()

    while queue:
        (cost, node, path) = heapq.heappop(queue)
        if node in visited:
            continue
        visited.add(node)
        path = path + [node]

        if node == end:
            return cost, path

        for neighbor, weight in graph.edges.get(node, []):
            if neighbor not in visited:
                heapq.heappush(queue, (cost + weight, neighbor, path))
    
    return float("inf"), []

# =======================
# 6. TSP BRUTE FORCE
# =======================
def tsp_brute_force(graph, start):
    nodes = list(graph.edges.keys())
    nodes.remove(start)

    min_path = None
    min_cost = float('inf')

    for perm in itertools.permutations(nodes):
        path = [start] + list(perm) + [start]
        cost = 0
        valid = True

        for i in range(len(path) - 1):
            current = path[i]
            next_node = path[i + 1]
            neighbors = dict(graph.edges[current])
            if next_node in neighbors:
                cost += neighbors[next_node]
            else:
                valid = False
                break

        if valid and cost < min_cost:
            min_cost = cost
            min_path = path

    return min_cost, min_path

# =======================
# 7. MENU UTAMA
# =======================
def display_cities():
    print("\nDaftar Kota:")
    for i, city in enumerate(cities, 1):
        print(f"{i}. {city}")

def main_menu():
    while True:
        print("\n===== MENU GPS KOTA DI KOREA UTARA =====")
        print("1. Tampilkan semua kota")
        print("2. Jalankan Dijkstra (cari rute tercepat)")
        print("3. Jalankan TSP (rute keliling semua kota)")
        print("4. Lihat Struktur Graph")
        print("5. Tampilkan Visualisasi Graph")
        print("0. Keluar")
        
        choice = input("Pilih menu: ").strip()
        
        if choice == "1":
            display_cities()
            
        elif choice == "2":
            print("\n=== Dijkstra ===")
            display_cities()
            start = input("Masukkan kota asal: ").strip().title()
            end = input("Masukkan kota tujuan: ").strip().title()
            
            if start not in cities or end not in cities:
                print("Kota tidak valid!")
                continue
                
            total_cost, route = dijkstra(graph, start, end)
            if total_cost != float("inf"):
                print(f"\nJalur tercepat: {' → '.join(route)}")
                print(f"Total jarak: {total_cost} km")
            else:
                print("Tidak ditemukan jalur.")
                
        elif choice == "3":
            print("\n=== Traveling Salesman Problem ===")
            display_cities()
            start = input("Masukkan kota awal untuk TSP: ").strip().title()
            
            if start not in cities:
                print("Kota tidak valid!")
                continue
                
            cost, path = tsp_brute_force(graph, start)
            if path:
                print(f"\nRute TSP terbaik: {' → '.join(path)}")
                print(f"Total jarak: {cost} km")
            else:
                print("Tidak ditemukan rute TSP yang valid.")
                
        elif choice == "4":
            print("\nStruktur Graph:")
            graph.display()
            
        elif choice == "5":
            visualize_graph()
            
        elif choice == "0":
            print("Terima kasih telah menggunakan program ini.")
            break
            
        else:
            print("Pilihan tidak valid. Silakan coba lagi.")

if __name__ == "__main__":
    main_menu()
