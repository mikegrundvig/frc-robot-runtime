package com.michaelgrundvig.frc.toolconfigs.advantagescope;

import io.avaje.jsonb.Jsonb;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Writes meshes as a binary glTF 2.0 file (a {@code .glb}), by the Khronos specification, with no
 * glTF library: a 12-byte header, a JSON chunk describing the scene (written by avaje-jsonb), and a
 * binary chunk holding every corner's position and normal as little-endian 32-bit floats.
 *
 * <p>Each mesh is one node and one primitive of triangles (no index buffer: every corner is its
 * own, since faces are flat), with its own material, a flat color.
 *
 * <p>glTF's axes are the specification's: +y up, and an asset's front faces +z. So a point's robot
 * coordinates (forward, left, up) are written as glTF's (left, up, forward), and any glTF viewer
 * shows the robot upright. AdvantageScope turns it back with the rotations in the robot's
 * config.json: 90° about x, then 90° about z ({@link RobotConfigJson#GLTF_TO_ROBOT}).
 */
final class Glb {
  /** "glTF", the file's first four bytes, read little-endian. */
  static final int MAGIC = 0x46546C67;

  /** The version of the binary container, which is glTF 2's. */
  static final int VERSION = 2;

  /** "JSON", a chunk's type. */
  static final int JSON_CHUNK = 0x4E4F534A;

  /** "BIN" and a zero byte, a chunk's type. */
  static final int BINARY_CHUNK = 0x004E4942;

  /** The file's header: its magic, its version and its whole length, four bytes each. */
  private static final int HEADER_BYTES = 12;

  /** What comes before each chunk's data: its length and its type, four bytes each. */
  private static final int CHUNK_HEADER_BYTES = 8;

  /** Each chunk's data is padded to a whole number of this many bytes. */
  private static final int CHUNK_ALIGNMENT_BYTES = 4;

  // glTF's codes, which are OpenGL's, from the specification.

  /** An accessor's component type: 32-bit floats. */
  private static final int FLOAT = 5126;

  /** A buffer view's target: data for corners, as opposed to indices. */
  private static final int ARRAY_BUFFER = 34962;

  /** A primitive's mode: every three corners make a triangle. */
  private static final int TRIANGLES = 4;

  private Glb() {}

  /**
   * A whole {@code .glb} file.
   *
   * @param generator what made it, and that it's generated: glTF's place for saying so
   */
  static byte[] write(String generator, List<Mesh> meshes) {
    Scene scene = new Scene();
    for (Mesh mesh : meshes) {
      boolean hasNothingToDraw = mesh.vertexCount() == 0;
      if (!hasNothingToDraw) {
        scene.add(mesh);
      }
    }
    String json = Jsonb.instance().type(Object.class).toJson(scene.document(generator));
    return pack(json.getBytes(StandardCharsets.UTF_8), scene.buffers, scene.bufferBytes);
  }

  /**
   * A glTF scene being put together, a mesh at a time. glTF describes a mesh in layers, each
   * referring to the next by its number in a list:
   *
   * <ul>
   *   <li>a node places a mesh in the scene;
   *   <li>a mesh has a primitive, which names a material and, for each of its attributes (its
   *       corners' positions, and their normals), an accessor;
   *   <li>an accessor says how to read a buffer view as numbers: here, three floats a corner;
   *   <li>a buffer view is a stretch of the file's one binary buffer.
   * </ul>
   *
   * <p>The lists are kept in the order things are added, which is the order they are written in.
   */
  private static final class Scene {
    private final List<Map<String, Object>> nodes = new ArrayList<>();
    private final List<Map<String, Object>> meshes = new ArrayList<>();
    private final List<Map<String, Object>> materials = new ArrayList<>();
    private final List<Map<String, Object>> accessors = new ArrayList<>();
    private final List<Map<String, Object>> bufferViews = new ArrayList<>();

    /** The materials written so far, each at its number in {@link #materials}. */
    private final List<Mesh.Material> materialsWritten = new ArrayList<>();

    /** The binary buffer's contents, in order: one array for each buffer view. */
    private final List<float[]> buffers = new ArrayList<>();

    /** How long the binary buffer is so far, in bytes. */
    private int bufferBytes;

    void add(Mesh mesh) {
      float[] positions = toGltf(mesh.positions());
      float[] normals = toGltf(mesh.normals());
      int material = numberOf(mesh.material());

      Map<String, Object> attributes = new LinkedHashMap<>();
      attributes.put("POSITION", accessors.size());
      accessors.add(positionAccessor(bufferViews.size(), mesh.vertexCount(), positions));
      addToBuffer(positions);
      attributes.put("NORMAL", accessors.size());
      accessors.add(accessor(bufferViews.size(), mesh.vertexCount()));
      addToBuffer(normals);

      Map<String, Object> primitive = new LinkedHashMap<>();
      primitive.put("attributes", attributes);
      primitive.put("material", material);
      primitive.put("mode", TRIANGLES);
      Map<String, Object> gltfMesh = new LinkedHashMap<>();
      gltfMesh.put("name", mesh.name());
      gltfMesh.put("primitives", List.of(primitive));
      Map<String, Object> node = new LinkedHashMap<>();
      node.put("name", mesh.name());
      node.put("mesh", meshes.size());
      meshes.add(gltfMesh);
      nodes.add(node);
    }

    /** A material's number, writing the material first if no mesh before has used it. */
    private int numberOf(Mesh.Material material) {
      int written = materialsWritten.indexOf(material);
      if (written >= 0) {
        return written;
      }
      materialsWritten.add(material);
      materials.add(Glb.material(material));
      return materialsWritten.size() - 1;
    }

    /** Adds numbers to the end of the binary buffer, with a buffer view over them. */
    private void addToBuffer(float[] numbers) {
      bufferViews.add(view(bufferBytes, numbers.length));
      bufferBytes += Float.BYTES * numbers.length;
      buffers.add(numbers);
    }

    /**
     * The scene as glTF's JSON. A scene with no meshes has none of the lists that describe one: the
     * specification does not allow them to be empty.
     */
    Map<String, Object> document(String generator) {
      Map<String, Object> asset = new LinkedHashMap<>();
      asset.put("version", "2.0");
      asset.put("generator", generator);
      List<Integer> everyNode = new ArrayList<>();
      for (int node = 0; node < nodes.size(); node++) {
        everyNode.add(node);
      }
      Map<String, Object> gltf = new LinkedHashMap<>();
      gltf.put("asset", asset);
      gltf.put("scene", 0);
      gltf.put("scenes", List.of(Map.of("nodes", everyNode)));
      gltf.put("nodes", nodes);
      if (!meshes.isEmpty()) {
        gltf.put("meshes", meshes);
        gltf.put("materials", materials);
        gltf.put("accessors", accessors);
        gltf.put("bufferViews", bufferViews);
        gltf.put("buffers", List.of(Map.of("byteLength", bufferBytes)));
      }
      return gltf;
    }
  }

  /**
   * Points or directions in the robot's frame, as glTF has them. Each is three numbers, (forward,
   * left, up), and is written as glTF's (left, up, forward).
   */
  private static float[] toGltf(float[] robot) {
    float[] gltf = new float[robot.length];
    for (int at = 0; at < robot.length; at += 3) {
      float forward = robot[at];
      float left = robot[at + 1];
      float up = robot[at + 2];
      gltf[at] = left;
      gltf[at + 1] = up;
      gltf[at + 2] = forward;
    }
    return gltf;
  }

  /**
   * A flat color as a glTF material: not metallic and fully rough, so it does not shine, and drawn
   * from both sides. One that is not fully opaque is blended with what is behind it.
   */
  private static Map<String, Object> material(Mesh.Material material) {
    Map<String, Object> pbr = new LinkedHashMap<>();
    pbr.put(
        "baseColorFactor",
        List.of(material.red(), material.green(), material.blue(), material.alpha()));
    pbr.put("metallicFactor", 0.0);
    pbr.put("roughnessFactor", 1.0);
    Map<String, Object> gltf = new LinkedHashMap<>();
    gltf.put("name", material.name());
    gltf.put("pbrMetallicRoughness", pbr);
    boolean seeThrough = material.alpha() < 1;
    if (seeThrough) {
      gltf.put("alphaMode", "BLEND");
    }
    gltf.put("doubleSided", true);
    return gltf;
  }

  /** An accessor that reads a buffer view as three floats a corner. */
  private static Map<String, Object> accessor(int bufferView, int corners) {
    Map<String, Object> accessor = new LinkedHashMap<>();
    accessor.put("bufferView", bufferView);
    accessor.put("componentType", FLOAT);
    accessor.put("count", corners);
    accessor.put("type", "VEC3");
    return accessor;
  }

  /**
   * Positions carry their bounds, as the specification requires, each the exact float the data
   * holds: a viewer checks them against the data.
   */
  private static Map<String, Object> positionAccessor(
      int bufferView, int corners, float[] positions) {
    List<Double> min = new ArrayList<>();
    List<Double> max = new ArrayList<>();
    for (int axis = 0; axis < 3; axis++) {
      float least = Float.POSITIVE_INFINITY;
      float most = Float.NEGATIVE_INFINITY;
      for (int at = axis; at < positions.length; at += 3) {
        least = Math.min(least, positions[at]);
        most = Math.max(most, positions[at]);
      }
      min.add((double) least);
      max.add((double) most);
    }
    Map<String, Object> accessor = accessor(bufferView, corners);
    accessor.put("min", min);
    accessor.put("max", max);
    return accessor;
  }

  /** A buffer view over some floats of the binary buffer, from an offset in bytes. */
  private static Map<String, Object> view(int byteOffset, int floats) {
    Map<String, Object> view = new LinkedHashMap<>();
    view.put("buffer", 0);
    view.put("byteOffset", byteOffset);
    view.put("byteLength", Float.BYTES * floats);
    view.put("target", ARRAY_BUFFER);
    return view;
  }

  /**
   * The header and the two chunks, each chunk padded to four bytes: the JSON with spaces, the
   * binary data with zeros, as the specification asks. A file with no binary data has no binary
   * chunk at all.
   *
   * @param buffers the binary data, in order
   * @param binaryBytes how long the binary data is, in bytes
   */
  private static byte[] pack(byte[] json, List<float[]> buffers, int binaryBytes) {
    int jsonPadded = padded(json.length);
    int binaryPadded = padded(binaryBytes);
    boolean hasBinary = binaryBytes > 0;
    int total = HEADER_BYTES + CHUNK_HEADER_BYTES + jsonPadded;
    if (hasBinary) {
      total += CHUNK_HEADER_BYTES + binaryPadded;
    }
    ByteBuffer out = ByteBuffer.allocate(total).order(ByteOrder.LITTLE_ENDIAN);
    out.putInt(MAGIC).putInt(VERSION).putInt(total);
    out.putInt(jsonPadded).putInt(JSON_CHUNK).put(json);
    for (int written = json.length; written < jsonPadded; written++) {
      out.put((byte) ' ');
    }
    if (hasBinary) {
      out.putInt(binaryPadded).putInt(BINARY_CHUNK);
      for (float[] buffer : buffers) {
        for (float value : buffer) {
          out.putFloat(value);
        }
      }
      // ByteBuffer.allocate gave zeros for the padding.
    }
    return out.array();
  }

  /** A length rounded up to a whole number of the bytes a chunk is aligned to. */
  private static int padded(int length) {
    int remainder = length % CHUNK_ALIGNMENT_BYTES;
    if (remainder == 0) {
      return length;
    }
    return length + CHUNK_ALIGNMENT_BYTES - remainder;
  }
}
