package com.michaelgrundvig.frc.toolconfigs.advantagescope;

import io.avaje.jsonb.Jsonb;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import org.jspecify.annotations.Nullable;

/**
 * Reads a binary glTF file back, and checks it against the parts of the glTF 2.0 specification a
 * viewer leans on: the header and chunks, every index pointing at something that exists, every
 * accessor inside its buffer view and every view inside the buffer, positions' bounds equal to the
 * data, normals a unit long, whole triangles. No glTF validator is available offline, so this
 * stands in for one; it reads only what the writer writes (float vectors, no indices).
 */
final class GlbReader {
  /** One mesh as read: its name, its material's name and color, and its corners. */
  record ReadMesh(String name, String material, List<Double> color, float[] positions) {}

  private final ByteBuffer file;
  private final List<String> problems = new ArrayList<>();
  private Map<String, Object> json = Map.of();
  private final List<ReadMesh> meshes = new ArrayList<>();

  private GlbReader(byte[] bytes) {
    file = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN);
    read();
  }

  static GlbReader read(byte[] bytes) {
    return new GlbReader(bytes);
  }

  /** What's wrong with the file, each in a sentence; empty for a valid file. */
  List<String> problems() {
    return problems;
  }

  /** The JSON chunk. */
  Map<String, Object> json() {
    return json;
  }

  List<ReadMesh> meshes() {
    return meshes;
  }

  /** A mesh by its name. */
  ReadMesh mesh(String name) {
    return meshes.stream()
        .filter(mesh -> mesh.name().equals(name))
        .findFirst()
        .orElseThrow(() -> new AssertionError("no mesh named " + name + " in " + meshes));
  }

  @SuppressWarnings("unchecked")
  private void read() {
    if (file.remaining() < 20) {
      problems.add("shorter than a header and a chunk");
      return;
    }
    check(file.getInt(0) == Glb.MAGIC, "the first four bytes aren't glTF's");
    check(file.getInt(4) == 2, "not glTF version 2: " + file.getInt(4));
    check(file.getInt(8) == file.capacity(), "the header's length isn't the file's");
    int jsonLength = file.getInt(12);
    check(file.getInt(16) == Glb.JSON_CHUNK, "the first chunk isn't JSON");
    check(jsonLength % 4 == 0, "the JSON chunk isn't padded to four bytes");
    String text = new String(file.array(), 20, jsonLength, StandardCharsets.UTF_8);
    json = (Map<String, Object>) Jsonb.instance().type(Object.class).fromJson(text);
    int binaryAt = 20 + jsonLength;
    int binaryLength = 0;
    if (binaryAt < file.capacity()) {
      binaryLength = file.getInt(binaryAt);
      check(file.getInt(binaryAt + 4) == Glb.BINARY_CHUNK, "the second chunk isn't binary");
      check(binaryLength % 4 == 0, "the binary chunk isn't padded to four bytes");
      check(binaryAt + 8 + binaryLength == file.capacity(), "the binary chunk isn't the rest");
    }

    Map<String, Object> asset = map(json.get("asset"));
    check("2.0".equals(asset.get("version")), "asset.version isn't 2.0");
    List<Object> scenes = list(json.get("scenes"));
    List<Object> nodes = list(json.get("nodes"));
    List<Object> gltfMeshes = list(json.get("meshes"));
    List<Object> materials = list(json.get("materials"));
    List<Object> accessors = list(json.get("accessors"));
    List<Object> views = list(json.get("bufferViews"));
    List<Object> buffers = list(json.get("buffers"));
    check(index(json.get("scene")) < scenes.size(), "the scene isn't one of the scenes");
    for (Object scene : scenes) {
      for (Object node : list(map(scene).get("nodes"))) {
        check(index(node) < nodes.size(), "a scene names a node that isn't there");
      }
    }
    if (!buffers.isEmpty()) {
      check(buffers.size() == 1, "a .glb has one buffer");
      check(
          index(map(buffers.get(0)).get("byteLength")) <= binaryLength,
          "the buffer is longer than the binary chunk");
    }
    for (Object view : views) {
      Map<String, Object> v = map(view);
      check(index(v.get("buffer")) == 0, "a view names a buffer that isn't there");
      check(
          index(v.get("byteOffset")) + index(v.get("byteLength"))
              <= index(map(buffers.get(0)).get("byteLength")),
          "a view runs past the buffer");
      check(index(v.get("byteOffset")) % 4 == 0, "a view isn't aligned to its floats");
    }
    for (Object node : nodes) {
      Map<String, Object> n = map(node);
      int meshIndex = index(n.get("mesh"));
      check(meshIndex < gltfMeshes.size(), "a node names a mesh that isn't there");
      Map<String, Object> mesh = map(gltfMeshes.get(meshIndex));
      for (Object primitiveValue : list(mesh.get("primitives"))) {
        Map<String, Object> primitive = map(primitiveValue);
        check(index(primitive.get("mode")) == 4, "a primitive isn't triangles");
        int materialIndex = index(primitive.get("material"));
        check(materialIndex < materials.size(), "a primitive names a material that isn't there");
        Map<String, Object> material = map(materials.get(materialIndex));
        List<Double> color =
            list(map(material.get("pbrMetallicRoughness")).get("baseColorFactor")).stream()
                .map(GlbReader::number)
                .toList();
        check(color.size() == 4, "a color isn't four numbers");
        Map<String, Object> attributes = map(primitive.get("attributes"));
        float[] positions = floats(accessors, views, binaryAt + 8, attributes.get("POSITION"));
        float[] normals = floats(accessors, views, binaryAt + 8, attributes.get("NORMAL"));
        check(positions.length == normals.length, "a mesh has a normal for each corner");
        check(positions.length % 9 == 0, "a mesh's corners aren't whole triangles");
        checkBounds(map(accessors.get(index(attributes.get("POSITION")))), positions);
        for (int i = 0; i < normals.length; i += 3) {
          double length =
              Math.sqrt(
                  normals[i] * normals[i]
                      + normals[i + 1] * normals[i + 1]
                      + normals[i + 2] * normals[i + 2]);
          check(Math.abs(length - 1) < 1e-4, "a normal isn't a unit long: " + length);
        }
        meshes.add(
            new ReadMesh(
                String.valueOf(mesh.get("name")),
                String.valueOf(material.get("name")),
                color,
                positions));
      }
    }
  }

  /** An accessor's floats, checked to lie inside its view. */
  private float[] floats(
      List<Object> accessors, List<Object> views, int binaryStart, @Nullable Object at) {
    Map<String, Object> accessor = map(accessors.get(index(at)));
    check(index(accessor.get("componentType")) == 5126, "an accessor isn't floats");
    check("VEC3".equals(accessor.get("type")), "an accessor isn't three-vectors");
    Map<String, Object> view = map(views.get(index(accessor.get("bufferView"))));
    int count = index(accessor.get("count"));
    int offset = accessor.containsKey("byteOffset") ? index(accessor.get("byteOffset")) : 0;
    check(
        offset + 12 * count <= index(view.get("byteLength")),
        "an accessor runs past its view: " + count + " corners");
    float[] values = new float[3 * count];
    int start = binaryStart + index(view.get("byteOffset")) + offset;
    for (int i = 0; i < values.length; i++) {
      values[i] = file.getFloat(start + 4 * i);
    }
    return values;
  }

  private void checkBounds(Map<String, Object> accessor, float[] positions) {
    List<Object> min = list(accessor.get("min"));
    List<Object> max = list(accessor.get("max"));
    for (int axis = 0; axis < 3; axis++) {
      float low = Float.POSITIVE_INFINITY;
      float high = Float.NEGATIVE_INFINITY;
      for (int i = axis; i < positions.length; i += 3) {
        low = Math.min(low, positions[i]);
        high = Math.max(high, positions[i]);
      }
      check((float) number(min.get(axis)) == low, "a position's minimum isn't the data's");
      check((float) number(max.get(axis)) == high, "a position's maximum isn't the data's");
    }
  }

  private void check(boolean ok, String problem) {
    if (!ok) {
      problems.add(problem);
    }
  }

  @SuppressWarnings("unchecked")
  static Map<String, Object> map(@Nullable Object value) {
    return value instanceof Map<?, ?> map ? (Map<String, Object>) map : new LinkedHashMap<>();
  }

  @SuppressWarnings("unchecked")
  static List<Object> list(@Nullable Object value) {
    return value instanceof List<?> list ? (List<Object>) list : List.of();
  }

  static double number(@Nullable Object value) {
    return ((Number) Objects.requireNonNull(value, "a number")).doubleValue();
  }

  private static int index(@Nullable Object value) {
    return value instanceof Number number ? number.intValue() : Integer.MAX_VALUE;
  }
}
